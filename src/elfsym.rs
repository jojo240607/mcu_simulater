//! ELF 符号地址解析——消除测试对固件内存布局的硬编码耦合。
//!
//! ## 为什么需要
//!
//! 测试要直读固件侧的全局量（`EST_STATE` / `SENSOR_SEQ` / `LOG_RING` …）做断言。
//! linker 只钉住 `.app_globals` 这样的段的**起始地址**，**段内各符号的顺序由链接器
//! 按代码自行决定**——固件一改（增删符号、改代码体积）顺序就变。实测一次 USB 相关
//! 改动让 `SENSOR_SEQ` 从 `0x200116DC` 移到 `0x200108C0`、`EST_STATE` 从
//! `0x2000F184` 移到 `0x2000F06C`，于是硬编码地址的测试静默读到垃圾，表现为
//! "采样率 0Hz"、"health=232"、"姿态四元数 w≈1e-19" 这类**假失败**——排查成本极高。
//!
//! 因此改为**从 ELF 符号表按名字解析**：地址随固件自动跟随，改动固件不再需要
//! 手工维护一串魔数。
//!
//! ## 实现
//!
//! 不引入任何依赖：手写 ELF32 小端 `.symtab`/`.strtab` 解析。Rust 静态量的符号名是
//! mangled 形式（如 `_RNvNtCsbs8ABJxAGNI_11flyctrl_app7flyctrl10SENSOR_SEQ.0`），
//! 故按**子串**匹配，取首个命中。

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

use crate::artifact;

/// 一个符号：名字 + 地址 + 大小。
#[derive(Debug, Clone)]
pub struct ElfSym {
    pub name: String,
    pub addr: u32,
    pub size: u32,
}

fn u16le(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn u32le(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

/// 解析 ELF32 小端的静态符号（`.symtab`）。失败返回空表（调用方据此报错）。
pub fn symbols(path: &Path) -> Vec<ElfSym> {
    let Ok(b) = std::fs::read(path) else {
        return Vec::new();
    };
    // ELF 头：magic + 32 位小端 + 32 位机（1 = 32-bit）
    if b.len() < 52 || &b[0..4] != b"\x7fELF" || b[4] != 1 || b[5] != 1 {
        return Vec::new();
    }
    let shoff = u32le(&b, 0x20) as usize;
    let shentsize = u16le(&b, 0x2E) as usize;
    let shnum = u16le(&b, 0x30) as usize;
    if shentsize < 40 || shoff == 0 {
        return Vec::new();
    }
    let mut out = Vec::new();
    for i in 0..shnum {
        let sh = shoff + i * shentsize;
        if sh + 40 > b.len() {
            break;
        }
        let sh_type = u32le(&b, sh + 4);
        if sh_type != 2 {
            // 2 = SHT_SYMTAB
            continue;
        }
        let sh_offset = u32le(&b, sh + 16) as usize;
        let sh_size = u32le(&b, sh + 20) as usize;
        let sh_link = u32le(&b, sh + 24) as usize; // 关联的 strtab 节号
        if sh_link >= shnum {
            continue;
        }
        let str_sh = shoff + sh_link * shentsize;
        if str_sh + 40 > b.len() {
            continue;
        }
        let str_off = u32le(&b, str_sh + 16) as usize;
        let str_len = u32le(&b, str_sh + 20) as usize;
        let strtab = &b[str_off.min(b.len())..(str_off + str_len).min(b.len())];
        // 每条 Elf32_Sym 16 字节：st_name(4) st_value(4) st_size(4) st_info(1) ...
        let mut off = sh_offset;
        let end = sh_offset + sh_size;
        while off + 16 <= end && off + 16 <= b.len() {
            let st_name = u32le(&b, off) as usize;
            let st_value = u32le(&b, off + 4);
            let st_size = u32le(&b, off + 8);
            if st_name != 0 && st_name < strtab.len() {
                let rest = &strtab[st_name..];
                let n = rest.iter().position(|c| *c == 0).unwrap_or(rest.len());
                let name = String::from_utf8_lossy(&rest[..n]).into_owned();
                out.push(ElfSym {
                    name,
                    addr: st_value,
                    size: st_size,
                });
            }
            off += 16;
        }
    }
    out
}

/// flyctrl 固件 ELF 路径（符号解析的对象）。
///
/// 与 app.bin 同源：`flyctrl/app.elf`。允许 env `JOC_APP_FLYCTRL_ELF` 覆盖
/// （与 `scripts/integrate.sh` 的产物导出约定一致）。
pub fn flyctrl_app_elf() -> PathBuf {
    if let Ok(p) = std::env::var("JOC_APP_FLYCTRL_ELF") {
        return PathBuf::from(p);
    }
    let mut cands: Vec<PathBuf> = Vec::new();
    if let Ok(man) = std::env::var("CARGO_MANIFEST_DIR") {
        cands.push(PathBuf::from(&man).join("../flyctrl/app.elf"));
    }
    cands.push(PathBuf::from("../flyctrl/app.elf"));
    cands.push(PathBuf::from("flyctrl/app.elf"));
    for c in &cands {
        if c.exists() {
            return c.clone();
        }
    }
    // 与 app.bin 同目录兜底（app.bin 的解析已含环境变量与历史路径）
    let bin = artifact::flyctrl_real_app_bin();
    if let Some(d) = bin.parent() {
        let e = d.join("app.elf");
        if e.exists() {
            return e;
        }
    }
    cands.remove(0)
}

fn cache() -> &'static Mutex<HashMap<String, (u32, u32)>> {
    static C: OnceLock<Mutex<HashMap<String, (u32, u32)>>> = OnceLock::new();
    C.get_or_init(|| Mutex::new(HashMap::new()))
}

fn all() -> &'static Vec<ElfSym> {
    static S: OnceLock<Vec<ElfSym>> = OnceLock::new();
    S.get_or_init(|| symbols(&flyctrl_app_elf()))
}

/// 按名字**子串**查 flyctrl 固件符号地址（首次查询时解析 ELF 并缓存）。
///
/// 查不到时 panic——这是测试脚手架的错误（固件改动使符号消失/改名），
/// 应在测试里立刻暴露，而不是静默用 0 地址读到垃圾。
pub fn app_sym(needle: &str) -> u32 {
    if let Some((a, _)) = cache().lock().unwrap().get(needle) {
        return *a;
    }
    let syms = all();
    let hit = syms
        .iter()
        .find(|s| s.name.contains(needle) && s.addr != 0)
        .unwrap_or_else(|| {
            panic!(
                "ELF 符号未找到: {needle:?}（ELF={}，共 {} 个符号）——固件符号改名/被裁掉了？",
                flyctrl_app_elf().display(),
                syms.len()
            )
        });
    let mut g = cache().lock().unwrap();
    g.insert(needle.to_string(), (hit.addr, hit.size));
    hit.addr
}

/// 同 [`app_sym`]，但返回 `(地址, 大小)`。
pub fn app_sym_size(needle: &str) -> (u32, u32) {
    let a = app_sym(needle);
    let sz = all()
        .iter()
        .find(|s| s.addr == a)
        .map(|s| s.size)
        .unwrap_or(0);
    (a, sz)
}

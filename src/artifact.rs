//! 联调固件/应用产物的路径解析——消除测试与工具中的机器硬编码绝对路径。
//!
//! 背景：`fc-umbrella` 壳工程以 submodule 纳入 9 个仓库，联调测试（mcu_simulater
//! `tests/x_*.rs`、fly-sim-server 等）需要加载跨仓库产物：
//!
//! | 产物 | 产出来源 |
//! |---|---|
//! | joc-base minimal ELF（`stm32f407_minimal.elf`） | `joc-base` 构建（build_hil / build_rel 配置等价，见 docs） |
//! | `flyctrl/app.bin` | `flyctrl` 构建（real-sensors / hil feature） |
//! | `joc-drvtest-app/app.bin` | `joc-drvtest-app` 构建 |
//! | `joc-rtos-app-sdk/app.bin` | `joc-rtos-app-sdk` 构建 |
//!
//! 解析优先级（每项产物一致）：
//! 1. **环境变量显式指定**（`JOC_BASE_ELF` / `JOC_APP_FLYCTRL` / `JOC_APP_DRVTEST` /
//!    `JOC_APP_SDK`）——`scripts/integrate.sh` 一键联调即导出这些变量；
//! 2. **壳工程规范布局**：本仓库上一级目录即壳工程根，按 `../joc-base/build_hil/...`、
//!    `../flyctrl/app.bin` 等相对位置查找（取首个存在的候选）；
//! 3. **历史开发机路径** `/home/ubuntu/work/...`（仅当存在时兜底，兼容旧工作区）。

use std::path::{Path, PathBuf};

/// 环境变量：joc-base minimal ELF（build_hil / build_rel 均可）。
pub const ENV_JOC_BASE_ELF: &str = "JOC_BASE_ELF";
/// 环境变量：flyctrl app.bin。
pub const ENV_APP_FLYCTRL: &str = "JOC_APP_FLYCTRL";
/// 环境变量：joc-drvtest-app app.bin。
pub const ENV_APP_DRVTEST: &str = "JOC_APP_DRVTEST";
/// 环境变量：joc-rtos-app-sdk app.bin。
pub const ENV_APP_SDK: &str = "JOC_APP_SDK";

/// 历史开发机布局（旧工作区兜底路径）。
const LEGACY_ROOT: &str = "/home/ubuntu/work";

/// 壳工程根：本仓库（mcu_simulater）的上一级目录，即 fc-umbrella 子模块标准布局。
pub fn umbrella_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."))
}

/// 解析一条产物路径。`canonical_rel` 为壳工程规范布局下的相对路径候选
/// （按序取首个存在者）；`legacy` 为历史布局路径（绝对或相对 root 均可，存在时兜底）。
fn resolve_in(root: &Path, env_key: &str, canonical_rel: &[&str], legacy: &str) -> PathBuf {
    if let Ok(v) = std::env::var(env_key) {
        return PathBuf::from(v);
    }
    for rel in canonical_rel {
        let p = root.join(rel);
        if p.exists() {
            return p;
        }
    }
    let legacy_p = root.join(legacy);
    if legacy_p.exists() {
        return legacy_p;
    }
    // 全部不存在：返回规范布局首个候选，报错信息可指引构建位置。
    root.join(canonical_rel[0])
}

fn resolve(env_key: &str, canonical_rel: &[&str], legacy_rel: &str) -> PathBuf {
    resolve_in(&umbrella_root(), env_key, canonical_rel, &format!("{LEGACY_ROOT}/{legacy_rel}"))
}

/// joc-base minimal ELF（`stm32f407_minimal.elf`）。
pub fn joc_base_elf() -> PathBuf {
    resolve(
        ENV_JOC_BASE_ELF,
        &[
            "joc-base/build_hil/stm32f407_minimal.elf",
            "joc-base/build_rel/stm32f407_minimal.elf",
        ],
        "joc-base/build_rel/stm32f407_minimal.elf",
    )
}

/// joc-base 构建目录（`jos_diag_out.txt` 等诊断输出的落点，与所用 ELF 同目录）。
pub fn joc_base_build_dir() -> PathBuf {
    joc_base_elf()
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| PathBuf::from("."))
}

/// flyctrl 固件 app.bin。
pub fn flyctrl_app_bin() -> PathBuf {
    resolve(ENV_APP_FLYCTRL, &["flyctrl/app.bin"], "flyctrl/app.bin")
}

/// joc-drvtest-app 固件 app.bin。
pub fn drvtest_app_bin() -> PathBuf {
    resolve(
        ENV_APP_DRVTEST,
        &["joc-drvtest-app/app.bin"],
        "joc-drvtest-app/app.bin",
    )
}

/// joc-rtos-app-sdk 固件 app.bin。
pub fn sdk_app_bin() -> PathBuf {
    resolve(
        ENV_APP_SDK,
        &["joc-rtos-app-sdk/app.bin"],
        "joc-rtos-app-sdk/app.bin",
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn tmp_root() -> PathBuf {
        let d = std::env::temp_dir().join(format!(
            "mcu_artifact_test_{}_{}",
            std::process::id(),
            std::thread::current().name().unwrap_or("t")
        ));
        let _ = fs::remove_dir_all(&d);
        fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn env_wins_over_everything() {
        // 用独立测试专用的 key，避免与其他测试/解析器共享 env 造成竞态。
        const KEY: &str = "JOC_ARTIFACT_TEST_ENV_ONLY";
        let root = tmp_root();
        // 规范布局与 legacy 均造出来存在，env 仍应优先（哪怕指向不存在的文件）。
        let canon = root.join("joc-base/build_hil/x.elf");
        fs::create_dir_all(canon.parent().unwrap()).unwrap();
        fs::write(&canon, b"elf").unwrap();
        unsafe {
            std::env::set_var(KEY, "/tmp/nonexistent-env.elf");
        }
        let got = resolve_in(&root, KEY, &["joc-base/build_hil/x.elf"], "joc-base/build_rel/x.elf");
        assert_eq!(got, PathBuf::from("/tmp/nonexistent-env.elf"));
        unsafe {
            std::env::remove_var(KEY);
        }
    }

    #[test]
    fn canonical_existing_wins_over_legacy() {
        let root = tmp_root();
        let canon = root.join("joc-base/build_hil/stm32f407_minimal.elf");
        fs::create_dir_all(canon.parent().unwrap()).unwrap();
        fs::write(&canon, b"elf").unwrap();
        let legacy = root.join("joc-base/build_rel/stm32f407_minimal.elf");
        fs::create_dir_all(legacy.parent().unwrap()).unwrap();
        fs::write(&legacy, b"elf2").unwrap();
        let got = resolve_in(&root, "JOC_BASE_ELF_UNUSED", &["joc-base/build_hil/stm32f407_minimal.elf"], "joc-base/build_rel/stm32f407_minimal.elf");
        assert_eq!(got, canon);
    }

    #[test]
    fn legacy_fallback_when_canonical_missing() {
        let root = tmp_root();
        let legacy = root.join("joc-base/build_rel/stm32f407_minimal.elf");
        fs::create_dir_all(legacy.parent().unwrap()).unwrap();
        fs::write(&legacy, b"elf").unwrap();
        let got = resolve_in(&root, "JOC_BASE_ELF_UNUSED", &["joc-base/build_hil/stm32f407_minimal.elf"], "joc-base/build_rel/stm32f407_minimal.elf");
        assert_eq!(got, legacy);
    }

    #[test]
    fn none_exists_returns_first_canonical() {
        let root = tmp_root();
        let got = resolve_in(&root, "JOC_BASE_ELF_UNUSED", &["joc-base/build_hil/x.elf"], "joc-base/build_rel/x.elf");
        assert_eq!(got, root.join("joc-base/build_hil/x.elf"));
    }
}

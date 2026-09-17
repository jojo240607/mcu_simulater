//! 联调固件/应用产物的路径解析——消除测试与工具中的机器硬编码绝对路径。
//!
//! 背景：`fc-umbrella` 壳工程以 submodule 纳入 9 个仓库，联调测试（mcu_simulater
//! `tests/x_*.rs`、fly-sim-server 等）需要加载跨仓库产物：
//!
//! | 产物 | 产出来源 |
//! |---|---|
//! | joc-base minimal ELF（`stm32f407_minimal.elf`） | `joc-base` 构建（build_hil / build_rel 配置等价，见 docs） |
//! | `flyctrl/app.bin` | `flyctrl` 默认 feature 构建（env `JOC_APP_FLYCTRL`） |
//! | `flyctrl` real-sensors 变体 | `./scripts/build.sh real-sensors` → `/tmp/flyctrl_real.bin`（env `JOC_APP_FLYCTRL_REAL`） |
//! | `joc-drvtest-app/app.bin` | `joc-drvtest-app` 构建 |
//! | `joc-rtos-app-sdk/app.bin` | `joc-rtos-app-sdk` 构建 |
//!
//! 解析优先级（每项产物一致）：
//! 1. **环境变量显式指定**（`JOC_BASE_ELF` / `JOC_APP_FLYCTRL` / `JOC_APP_FLYCTRL_REAL` /
//!    `JOC_APP_DRVTEST` / `JOC_APP_SDK`）——`scripts/integrate.sh` 一键联调即导出这些变量；
//! 2. **壳工程规范布局**：本仓库上一级目录即壳工程根，按 `../joc-base/build_hil/...`、
//!    `../flyctrl/app.bin` 等相对位置查找（取首个存在的候选）；
//! 3. **历史开发机路径** `/home/ubuntu/work/...`（仅当存在时兜底，兼容旧工作区）。

use std::path::{Path, PathBuf};

/// 环境变量：joc-base minimal ELF（build_hil / build_rel 均可）。
pub const ENV_JOC_BASE_ELF: &str = "JOC_BASE_ELF";
/// 环境变量：flyctrl app.bin。
pub const ENV_APP_FLYCTRL: &str = "JOC_APP_FLYCTRL";
/// 环境变量：flyctrl real-sensors feature 固件 app.bin（见 [`flyctrl_real_app_bin`]）。
pub const ENV_APP_FLYCTRL_REAL: &str = "JOC_APP_FLYCTRL_REAL";
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

/// 解析一条产物路径。
///
/// - `canonical_rel`：壳工程规范布局（`canonical_root`，即 umbrella 根）下的
///   相对路径候选，按序取首个存在者；
/// - `legacy_rels`：历史开发机布局（`legacy_root`）下的相对路径候选，
///   仅当存在时兜底。
///
/// canonical 与 legacy 是两个独立根：即使同一相对路径在两个根下都存在，
/// 也按"先 canonical 后 legacy"的顺序各自独立判定。
fn resolve_in(
    canonical_root: &Path,
    legacy_root: &Path,
    env_key: &str,
    canonical_rel: &[&str],
    legacy_rels: &[&str],
) -> PathBuf {
    if let Ok(v) = std::env::var(env_key) {
        return PathBuf::from(v);
    }
    for rel in canonical_rel {
        let p = canonical_root.join(rel);
        if p.exists() {
            return p;
        }
    }
    for rel in legacy_rels {
        let p = legacy_root.join(rel);
        if p.exists() {
            return p;
        }
    }
    // 全部不存在：返回规范布局首个候选，报错信息可指引构建位置。
    canonical_root.join(canonical_rel[0])
}

fn resolve(env_key: &str, canonical_rel: &[&str], legacy_rels: &[&str]) -> PathBuf {
    resolve_in(
        &umbrella_root(),
        Path::new(LEGACY_ROOT),
        env_key,
        canonical_rel,
        legacy_rels,
    )
}

/// joc-base minimal ELF。
///
/// 产物命名：旧工作区/文档长期使用 `stm32f407_minimal.elf`；锁定源码 CMake
/// 产出 `jOS.elf`（同一固件），故候选两者都查。
///
/// 目录语义（历史实测结论）：
/// - **规范布局 `build_hil`**：integrate.sh firmware 在锁定源码上构建，对所有
///   测试自洽（x_fault_injection 等已实测通过）→ canonical 优先 build_hil；
/// - **历史开发机 legacy**：`build_rel`（旧工作区较新构建）行为正常；
///   `build_hil`（旧工作区较早构建）行为分裂（x_fault_injection 失败）→
///   legacy 兜底 **build_rel 优先**，避免踩到旧的 build_hil 固件。
pub fn joc_base_elf() -> PathBuf {
    resolve(
        ENV_JOC_BASE_ELF,
        &[
            "joc-base/build_hil/stm32f407_minimal.elf",
            "joc-base/build_hil/jOS.elf",
            "joc-base/build_rel/stm32f407_minimal.elf",
            "joc-base/build_rel/jOS.elf",
        ],
        &[
            "joc-base/build_rel/stm32f407_minimal.elf",
            "joc-base/build_rel/jOS.elf",
            "joc-base/build_hil/stm32f407_minimal.elf",
            "joc-base/build_hil/jOS.elf",
        ],
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
    resolve(ENV_APP_FLYCTRL, &["flyctrl/app.bin"], &["flyctrl/app.bin"])
}

/// flyctrl **real-sensors feature** 固件 app.bin。
///
/// real-sensors 与默认/hil 是**两个不同产物**，因此独立于 [`flyctrl_app_bin`]
/// （后者经 `JOC_APP_FLYCTRL`，HIL 用）：
///
/// 解析优先级：
/// 1. env `JOC_APP_FLYCTRL_REAL`；
/// 2. `./scripts/build.sh real-sensors` 的约定落点 `/tmp/flyctrl_real.bin`
///    （README / docs / `fly-sim-server` 共用的产物名）；
/// 3. 壳工程内 `flyctrl/app_real.bin`（若产物构建进规范布局）；
/// 4. 全不存在时返回约定落点，报错信息指向构建命令。
pub fn flyctrl_real_app_bin() -> PathBuf {
    resolve(
        ENV_APP_FLYCTRL_REAL,
        &["/tmp/flyctrl_real.bin", "flyctrl/app_real.bin"],
        &["/tmp/flyctrl_real.bin"],
    )
}

/// joc-drvtest-app 固件 app.bin。
pub fn drvtest_app_bin() -> PathBuf {
    resolve(
        ENV_APP_DRVTEST,
        &["joc-drvtest-app/app.bin"],
        &["joc-drvtest-app/app.bin"],
    )
}

/// joc-rtos-app-sdk 固件 app.bin。
pub fn sdk_app_bin() -> PathBuf {
    resolve(
        ENV_APP_SDK,
        &["joc-rtos-app-sdk/app.bin"],
        &["joc-rtos-app-sdk/app.bin"],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn tmp_root(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!(
            "mcu_artifact_test_{}_{}_{}",
            std::process::id(),
            std::thread::current().name().unwrap_or("t"),
            tag
        ));
        let _ = fs::remove_dir_all(&d);
        fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn env_wins_over_everything() {
        // 用独立测试专用的 key，避免与其他测试/解析器共享 env 造成竞态。
        const KEY: &str = "JOC_ARTIFACT_TEST_ENV_ONLY";
        let canon_root = tmp_root("env_c");
        let legacy_root = tmp_root("env_l");
        // 规范布局与 legacy 均造出来存在，env 仍应优先（哪怕指向不存在的文件）。
        let canon = canon_root.join("joc-base/build_hil/x.elf");
        fs::create_dir_all(canon.parent().unwrap()).unwrap();
        fs::write(&canon, b"elf").unwrap();
        unsafe {
            std::env::set_var(KEY, "/tmp/nonexistent-env.elf");
        }
        let got = resolve_in(
            &canon_root,
            &legacy_root,
            KEY,
            &["joc-base/build_hil/x.elf"],
            &["joc-base/build_rel/x.elf"],
        );
        assert_eq!(got, PathBuf::from("/tmp/nonexistent-env.elf"));
        unsafe {
            std::env::remove_var(KEY);
        }
    }

    #[test]
    fn canonical_existing_wins_over_legacy() {
        let canon_root = tmp_root("c_wins_c");
        let legacy_root = tmp_root("c_wins_l");
        let canon = canon_root.join("joc-base/build_hil/stm32f407_minimal.elf");
        fs::create_dir_all(canon.parent().unwrap()).unwrap();
        fs::write(&canon, b"elf").unwrap();
        // legacy 同相对路径也存在（旧工作区同一目录名），仍应取 canonical。
        let legacy = legacy_root.join("joc-base/build_hil/stm32f407_minimal.elf");
        fs::create_dir_all(legacy.parent().unwrap()).unwrap();
        fs::write(&legacy, b"elf2").unwrap();
        let got = resolve_in(
            &canon_root,
            &legacy_root,
            "JOC_BASE_ELF_UNUSED",
            &["joc-base/build_hil/stm32f407_minimal.elf"],
            &["joc-base/build_hil/stm32f407_minimal.elf"],
        );
        assert_eq!(got, canon);
    }

    #[test]
    fn legacy_build_rel_preferred_over_old_build_hil() {
        // 历史开发机同时存在 build_hil（旧构建，行为分裂）与 build_rel（较新，
        // 行为正常），且 canonical 布局未构建——legacy 兜底须 build_rel 优先，
        // 避开旧 build_hil 固件。
        let canon_root = tmp_root("rel_c");
        let legacy_root = tmp_root("rel_l");
        let rel = legacy_root.join("joc-base/build_rel/stm32f407_minimal.elf");
        fs::create_dir_all(rel.parent().unwrap()).unwrap();
        fs::write(&rel, b"rel").unwrap();
        let hil = legacy_root.join("joc-base/build_hil/stm32f407_minimal.elf");
        fs::create_dir_all(hil.parent().unwrap()).unwrap();
        fs::write(&hil, b"hil").unwrap();
        let got = resolve_in(
            &canon_root,
            &legacy_root,
            "JOC_BASE_ELF_UNUSED",
            &["joc-base/build_hil/stm32f407_minimal.elf", "joc-base/build_hil/jOS.elf", "joc-base/build_rel/stm32f407_minimal.elf"],
            &["joc-base/build_rel/stm32f407_minimal.elf", "joc-base/build_rel/jOS.elf", "joc-base/build_hil/stm32f407_minimal.elf"],
        );
        assert_eq!(got, rel);
    }

    #[test]
    fn legacy_jos_elf_fallback() {
        // 锁定源码产物名 jOS.elf：legacy 侧 build_rel 只留 jOS.elf 时也应命中。
        let canon_root = tmp_root("jos_c");
        let legacy_root = tmp_root("jos_l");
        let jos = legacy_root.join("joc-base/build_rel/jOS.elf");
        fs::create_dir_all(jos.parent().unwrap()).unwrap();
        fs::write(&jos, b"elf").unwrap();
        let got = resolve_in(
            &canon_root,
            &legacy_root,
            "JOC_BASE_ELF_UNUSED",
            &["joc-base/build_hil/stm32f407_minimal.elf", "joc-base/build_hil/jOS.elf"],
            &["joc-base/build_rel/stm32f407_minimal.elf", "joc-base/build_rel/jOS.elf"],
        );
        assert_eq!(got, jos);
    }

    #[test]
    fn legacy_fallback_when_canonical_missing() {
        let canon_root = tmp_root("fall_c");
        let legacy_root = tmp_root("fall_l");
        let legacy = legacy_root.join("joc-base/build_rel/stm32f407_minimal.elf");
        fs::create_dir_all(legacy.parent().unwrap()).unwrap();
        fs::write(&legacy, b"elf").unwrap();
        let got = resolve_in(
            &canon_root,
            &legacy_root,
            "JOC_BASE_ELF_UNUSED",
            &["joc-base/build_hil/stm32f407_minimal.elf"],
            &["joc-base/build_rel/stm32f407_minimal.elf"],
        );
        assert_eq!(got, legacy);
    }

    #[test]
    fn none_exists_returns_first_canonical() {
        let canon_root = tmp_root("none_c");
        let legacy_root = tmp_root("none_l");
        let got = resolve_in(
            &canon_root,
            &legacy_root,
            "JOC_BASE_ELF_UNUSED",
            &["joc-base/build_hil/x.elf"],
            &["joc-base/build_rel/x.elf"],
        );
        assert_eq!(got, canon_root.join("joc-base/build_hil/x.elf"));
    }
}

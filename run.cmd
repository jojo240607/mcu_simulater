@echo off
rem run cfg-driven simulator (cfg-run). Enc: plain ASCII only (no BOM) so it runs under any OEM codepage.
rem Usage:
rem   run.cmd                    -> use .\run.cfg in this folder
rem   run.cmd path\to\board.cfg  -> use that config instead
setlocal

if not defined LIBCLANG_PATH set "LIBCLANG_PATH=D:\soft\llvm\bin"

set "CFG=%~dp0run.cfg"
if not "%~1"=="" set "CFG=%~1"
if not exist "%CFG%" (
  echo [err] config not found: %CFG%
  exit /b 1
)

echo == cfg-run file=%CFG% ==
cargo run --release --offline --bin cfg-run -- --cfg "%CFG%"

endlocal

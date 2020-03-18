use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let memfile_bytes = include_bytes!("stm32h743_memory.x");

    // Put the linker script somewhere the linker can find it
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memfile_bytes)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
}

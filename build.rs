extern crate gcc;


fn main() {
    build_clib();

    println!("cargo:rustc-link-lib=static=poly2tri");
}

fn build_clib() {
    let mut cfg = gcc::Config::new();
    cfg.cpp(true);
    cfg.include("vendor/poly2tri-cpp/poly2tri");
    cfg.file("src/binding.cpp");
    cfg.compile("libpoly2tri.a");
}

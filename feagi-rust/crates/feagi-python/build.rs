fn main() {
    pyo3_build_config::add_extension_module_link_args();
    
    // Windows-specific: Link against advapi32 for ZMQ security descriptor functions
    #[cfg(target_os = "windows")]
    {
        println!("cargo:rustc-link-lib=advapi32");
    }
}





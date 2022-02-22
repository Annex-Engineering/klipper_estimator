use anyhow::Result;

fn main() -> Result<()> {
    println!("cargo:rerun-if-env-changed=TOOL_VERSION");

    let version = match dbg!(std::env::var("TOOL_VERSION")) {
        Ok(value) => value,
        _ => {
            let dir = std::env::current_dir()?;
            let repo = git2::Repository::discover(dir)?;
            let mut desc_opts = git2::DescribeOptions::new();
            desc_opts.describe_tags();
            let desc = repo.describe(&desc_opts)?;
            let mut fmt_opts = git2::DescribeFormatOptions::new();
            fmt_opts.dirty_suffix("-dirty");
            desc.format(Some(&fmt_opts))?
        }
    };

    println!("cargo:rustc-env=TOOL_VERSION={version}");

    Ok(())
}

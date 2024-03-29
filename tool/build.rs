use anyhow::Result;

fn describe_git() -> Result<String> {
    let dir = std::env::current_dir()?;
    let repo = git2::Repository::discover(dir)?;
    let mut desc_opts = git2::DescribeOptions::new();
    desc_opts.describe_tags();
    let desc = repo.describe(&desc_opts)?;
    let mut fmt_opts = git2::DescribeFormatOptions::new();
    fmt_opts.dirty_suffix("-dirty");
    Ok(desc.format(Some(&fmt_opts))?)
}

fn main() -> Result<()> {
    println!("cargo:rerun-if-env-changed=TOOL_VERSION");

    let version = std::env::var("TOOL_VERSION")
        .or_else(|_| describe_git())
        .unwrap_or_else(|_| "unknown".into());

    println!("cargo:rustc-env=TOOL_VERSION={version}");

    Ok(())
}

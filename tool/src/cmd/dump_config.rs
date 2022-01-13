use clap::Parser;

use crate::Opts;

#[derive(Parser, Debug)]
pub struct DumpConfigCmd;

impl DumpConfigCmd {
    pub fn run(&self, opts: &Opts) {
        let _ = serde_json::to_writer_pretty(std::io::stdout(), &opts.printer_limits());
    }
}

use std::error::Error;

use lib_klipper::planner::{Planner, PrinterConfiguration, PrinterLimits};

use clap::Parser;
use once_cell::sync::OnceCell;

#[macro_use]
extern crate lazy_static;

mod cmd;

mod moonraker;

#[derive(Parser, Debug)]
#[clap(version = env!("TOOL_VERSION"), author = "Lasse Dalegaard <dalegaard@gmail.com>")]
pub struct Opts {
    #[clap(long = "config_moonraker_url")]
    config_moonraker: Option<String>,

    #[clap(long = "config_moonraker_api_key")]
    config_moonraker_api_key: Option<String>,

    #[clap(long = "config_file")]
    config_filename: Option<String>,

    #[clap(subcommand)]
    cmd: SubCommand,

    #[clap(skip)]
    config: OnceCell<PrinterConfiguration>,
}

impl Opts {
    fn printer_limits(&self) -> &PrinterConfiguration {
        match self.config.get() {
            Some(limits) => limits,
            None => match self.load_config() {
                Ok(limits) => {
                    let _ = self.config.set(limits);
                    self.config.get().unwrap()
                }
                Err(e) => {
                    eprintln!("Failed to load printer configuration: {}", e);
                    std::process::exit(1);
                }
            },
        }
    }

    fn load_config(&self) -> Result<PrinterConfiguration, Box<dyn Error>> {
        // Load config file
        // let mut limits = if let Some(filename) = &self.config_filename {
        //     let src = std::fs::read_to_string(filename)?;
        //     let mut limits: PrinterLimits = deser_hjson::from_str(&src)?;
        //
        //     // Do any fix-ups
        //     limits.set_square_corner_velocity(limits.square_corner_velocity);
        //
        //     Ok(PrinterConfiguration{
        //         limits,
        //         macros: vec![]
        //     })
        // } else {
        //     Ok(PrinterConfiguration::default())
        // };

        // Was moonraker config requested? If so, try to grab that first.
        if let Some(url) = &self.config_moonraker {
            let mut limits = PrinterLimits::default();
            let config = moonraker::moonraker_config(
                url,
                self.config_moonraker_api_key.as_deref(),
                &mut limits,
            )?;
            Ok(config)
        } else {
            Ok(PrinterConfiguration::default())
        }

        // Ok(limits)
    }

    fn make_planner(&self) -> Planner {
        Planner::from_limits(self.printer_limits().clone())
    }
}

#[derive(Parser, Debug)]
enum SubCommand {
    Estimate(cmd::estimate::EstimateCmd),
    DumpMoves(cmd::estimate::DumpMovesCmd),
    PostProcess(cmd::post_process::PostProcessCmd),
    DumpConfig(cmd::dump_config::DumpConfigCmd),
}

impl SubCommand {
    fn run(&self, opts: &Opts) {
        match self {
            Self::Estimate(i) => i.run(opts),
            Self::DumpMoves(i) => i.run(opts),
            Self::PostProcess(i) => i.run(opts),
            Self::DumpConfig(i) => i.run(opts),
        }
    }
}

fn main() {
    let opts = Opts::parse();
    opts.cmd.run(&opts);
}

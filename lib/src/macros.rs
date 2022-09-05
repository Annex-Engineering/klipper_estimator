use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use glam::DVec3 as Vec3;
use glam::{DVec4, Vec4Swizzles};

use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Clone)]
struct MacroToolheadPosition(Arc<Mutex<Position>>);

impl MacroToolheadPosition {
    fn update(&self, position: DVec4) {
        let mut toolhead_position = self.0.lock().unwrap();
        *toolhead_position = Position(position.xyz());
    }
}

struct PrinterObj {
    config_map: HashMap<String, minijinja::value::Value>,
    toolhead_position: MacroToolheadPosition,
    axis_maximum: Position,
}

impl std::fmt::Debug for PrinterObj {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PrinterObj")
            // skip serializing the configuration since usually it's a wall of text
            // nobody is interested in
            // .field("config_map", &self.config_map)
            .field("config_map", &"<...>")
            .field("toolhead_position", &self.toolhead_position)
            .field("axis_maximum", &self.axis_maximum)
            .finish()
    }
}

impl std::fmt::Display for PrinterObj {
    fn fmt(&self, _f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        unimplemented!()
    }
}

impl minijinja::value::Object for PrinterObj {
    fn get_attr(&self, name: &str) -> Option<minijinja::value::Value> {
        match name {
            "toolhead" => Some(minijinja::value::Value::from_serializable(
                &self.get_toolhead(),
            )),
            name => self.config_map.get(name).map(|v| v.to_owned()),
        }
    }

    fn attributes(&self) -> &[&str] {
        unimplemented!()
    }

    fn call_method(
        &self,
        state: &minijinja::State,
        name: &str,
        args: Vec<minijinja::value::Value>,
    ) -> Result<minijinja::value::Value, minijinja::Error> {
        let _state = state;
        let _args = args;
        Err(minijinja::Error::new(
            minijinja::ErrorKind::ImpossibleOperation,
            format!("object has no method named {}", name),
        ))
    }

    fn call(
        &self,
        state: &minijinja::State,
        args: Vec<minijinja::value::Value>,
    ) -> Result<minijinja::value::Value, minijinja::Error> {
        let _state = state;
        let _args = args;
        Err(minijinja::Error::new(
            minijinja::ErrorKind::ImpossibleOperation,
            "tried to call non callable object",
        ))
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct Position(Vec3);

impl Serialize for Position {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeMap;

        let mut map = serializer.serialize_map(Some(3))?;
        map.serialize_entry("x", &self.0.x)?;
        map.serialize_entry("y", &self.0.y)?;
        map.serialize_entry("z", &self.0.z)?;
        map.end()
    }
}

#[derive(Debug, Serialize)]
struct Toolhead {
    position: Position,
    axis_maximum: Position,
    homed_axes: String,
}

impl PrinterObj {
    fn get_toolhead(&self) -> Toolhead {
        let position = *self.toolhead_position.0.lock().unwrap();
        Toolhead {
            position,
            axis_maximum: self.axis_maximum,
            homed_axes: "xyz".to_owned(),
        }
    }

    fn new(
        config: &crate::planner::MacroConfiguration,
        toolhead_position: MacroToolheadPosition,
    ) -> Self {
        #[derive(Debug, Serialize)]
        struct Toolhead {
            position: Position,
            axis_maximum: Position,
            homed_axes: String,
        }

        let axis_maximum = Position(Vec3::new(
            config.config["stepper_x"]["position_max"].as_f64().unwrap() as f64,
            config.config["stepper_y"]["position_max"].as_f64().unwrap() as f64,
            config.config["stepper_z"]["position_max"].as_f64().unwrap() as f64,
        ));

        let mut cfg: HashMap<String, minijinja::value::Value> = config
            .config
            .iter()
            .map(|(k, v)| (k.to_owned(), minijinja::value::Value::from_serializable(v)))
            .collect();

        for mac in &config.macros {
            let mut name = "gcode_macro ".to_owned();
            name.push_str(&mac.name);
            let values = minijinja::value::Value::from_serializable(&mac.variables);
            cfg.insert(name, values);
        }

        PrinterObj {
            config_map: cfg,
            toolhead_position,
            axis_maximum,
        }
    }
}

fn read_macros<'a, I>(macros: I) -> minijinja::Source
where
    I: IntoIterator<Item = &'a GCodeMacro>,
{
    let re_braces = regex::Regex::new(r"(?x) ( \{\S\} ) | ( \{[^%] ) | ( [^%]\} )").unwrap();

    let mut src = minijinja::Source::new();

    for mac in macros {
        let res = re_braces
            .replace_all(&mac.gcode, |cap: &regex::Captures| {
                if let Some(m) = cap.get(1) {
                    "{".to_owned() + m.as_str() + "}"
                } else if let Some(m) = cap.get(2) {
                    "{".to_owned() + m.as_str()
                } else if let Some(m) = cap.get(3) {
                    m.as_str().to_owned() + "}"
                } else {
                    unreachable!()
                }
            })
            .to_ascii_lowercase();

        src.add_template(&mac.name, res).unwrap();
    }

    src
}

fn create_macro_environment(src: minijinja::Source) -> minijinja::Environment<'static> {
    fn any_id(
        _state: &minijinja::State,
        value: minijinja::value::Value,
    ) -> Result<minijinja::value::Value, minijinja::Error> {
        Ok(value)
    }

    fn min<Item>(_state: &minijinja::State, value: Vec<Item>) -> Result<Item, minijinja::Error>
    where
        Item: Ord,
    {
        value.into_iter().min().ok_or_else(|| {
            minijinja::Error::new(
                minijinja::ErrorKind::InvalidArguments,
                "can't compute minimum of empty list",
            )
        })
    }

    fn max<Item>(_state: &minijinja::State, value: Vec<Item>) -> Result<Item, minijinja::Error>
    where
        Item: Ord,
    {
        value.into_iter().max().ok_or_else(|| {
            minijinja::Error::new(
                minijinja::ErrorKind::InvalidArguments,
                "can't compute maximum of empty list",
            )
        })
    }

    fn action_nop(
        _state: &minijinja::State,
        _value: minijinja::value::Value,
    ) -> Result<(), minijinja::Error> {
        Ok(())
    }

    let mut env = minijinja::Environment::new();

    env.set_source(src);

    #[cfg(debug_assertions)]
    env.set_debug(true);

    env.add_filter("float", any_id);
    env.add_filter("int", any_id);
    env.add_filter("min", min::<i64>);
    env.add_filter("max", max::<i64>);

    env.add_function("action_respond_info", action_nop);
    env.add_function("action_raise_error", action_nop);

    env
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct GCodeMacro<S = serde_json::Value>
where
    S: Serialize,
{
    pub name: String,
    pub description: Option<String>,
    pub variables: HashMap<String, S>,
    pub gcode: String,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct MacroConfiguration {
    pub macros: Vec<GCodeMacro>,
    pub config: HashMap<String, serde_json::Value>,
}

#[derive(Debug)]
pub(crate) struct MacroManager {
    macro_variables: HashMap<String, HashMap<String, minijinja::value::Value>>,
    macro_environment: minijinja::Environment<'static>,
    toolhead_position: crate::macros::MacroToolheadPosition,
}

impl MacroManager {
    pub fn new(config: MacroConfiguration) -> Self {
        let toolhead_position = crate::macros::MacroToolheadPosition::default();
        let printer_obj = crate::macros::PrinterObj::new(&config, toolhead_position.clone());

        let src = crate::macros::read_macros(&config.macros);

        let mut macro_environment = crate::macros::create_macro_environment(src);
        macro_environment.add_global("printer", minijinja::value::Value::from_object(printer_obj));

        let macro_variables = config
            .macros
            .iter()
            .map(|mac| {
                (
                    mac.name.to_owned(),
                    mac.variables
                        .iter()
                        .map(|(k, v)| (k.to_owned(), minijinja::value::Value::from_serializable(v)))
                        .collect(),
                )
            })
            .collect();

        Self {
            macro_variables,
            macro_environment,
            toolhead_position,
        }
    }

    pub fn render_macro(
        &self,
        name: &str,
        params: crate::gcode::GCodeExtendedParams,
        position: DVec4,
    ) -> Option<String> {
        let vars = self.macro_variables.get(name)?.to_owned();

        // Update toolhead position shared with global PrinterObj
        self.toolhead_position.update(position);

        #[derive(Debug, Serialize)]
        struct MacroParams {
            params: crate::gcode::GCodeExtendedParams,
            #[serde(flatten)]
            variables: HashMap<String, minijinja::value::Value>,
        }

        let inner_params = MacroParams {
            params,
            variables: vars,
        };

        let template = self.macro_environment.get_template(name).ok()?;
        match template.render(inner_params) {
            Ok(rendered) => Some(rendered),
            Err(e) => {
                eprintln!("error during template: err={:?}", e);
                None
            }
        }
    }
}

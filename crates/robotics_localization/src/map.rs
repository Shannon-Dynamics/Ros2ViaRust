//! The landmark map, read from the simulation's own ground truth file.
//!
//! The single source of truth for where the pillars are is
//! `sim/webots/config/landmarks.yaml`, which is written next to the Webots world. Rather
//! than copy those coordinates into a second file that can drift, this module reads that
//! file directly.
//!
//! It is a deliberately small reader for the exact shape that file has, not a YAML
//! implementation: a `key:` line at column zero opens a block, and a landmark is one flow
//! mapping per line. Anything it does not understand is an error, never a silently skipped
//! line, because a landmark quietly dropped from the map is a localization bug that looks
//! like a filter bug.

use robotics_core::Se2;
use robotics_models::sensor::landmark::{Landmark, LandmarkMap};
use std::collections::HashMap;
use std::path::Path;

/// Errors of the map reader.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum MapError {
    /// The file could not be read.
    #[error("cannot read {path}: {reason}")]
    Io {
        /// Path that failed.
        path: String,
        /// Reason reported by the operating system.
        reason: String,
    },
    /// A line could not be parsed.
    #[error("line {line}: {what}")]
    Parse {
        /// 1-based line number.
        line: usize,
        /// What went wrong.
        what: String,
    },
    /// The file parsed but has no usable map.
    #[error("{0}")]
    Empty(&'static str),
}

/// One landmark as the file describes it.
#[derive(Debug, Clone, PartialEq)]
pub struct LandmarkEntry {
    /// Name in the file and in the Webots world, for example `landmark_a`.
    pub name: String,
    /// Position in the map frame (m).
    pub x: f64,
    /// Position in the map frame (m).
    pub y: f64,
    /// Radius of the pillar (m); the laser sees its surface, the map holds its centre.
    pub radius: f64,
}

/// The contents of `landmarks.yaml` that localization needs.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MapFile {
    /// Landmarks in file order; the index is the id used by [`LandmarkMap`].
    pub landmarks: Vec<LandmarkEntry>,
    /// Pose the robot is spawned at, when the file states one.
    pub spawn_pose: Option<Se2>,
}

fn parse_scalar(line: usize, key: &str, raw: &str) -> Result<f64, MapError> {
    raw.trim().parse::<f64>().map_err(|e| MapError::Parse {
        line,
        what: format!("{key}: {raw:?} is not a number ({e})"),
    })
}

/// Parse the `{a: 1, b: 2}` body of a flow mapping into its fields.
fn flow_fields(line: usize, body: &str) -> Result<HashMap<String, String>, MapError> {
    let mut out = HashMap::new();
    for pair in body.split(',') {
        let pair = pair.trim();
        if pair.is_empty() {
            continue;
        }
        let (k, v) = pair.split_once(':').ok_or_else(|| MapError::Parse {
            line,
            what: format!("{pair:?} is not a `key: value` pair"),
        })?;
        out.insert(k.trim().to_string(), v.trim().to_string());
    }
    Ok(out)
}

impl MapFile {
    /// Read and parse a map file.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, MapError> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| MapError::Io {
            path: path.display().to_string(),
            reason: e.to_string(),
        })?;
        Self::parse(&text)
    }

    /// Parse the contents of a map file.
    pub fn parse(text: &str) -> Result<Self, MapError> {
        let mut out = Self::default();
        let mut section = String::new();
        for (i, raw) in text.lines().enumerate() {
            let line = i + 1;
            let content = raw.split('#').next().unwrap_or("");
            if content.trim().is_empty() {
                continue;
            }
            let indent = content.len() - content.trim_start().len();
            let trimmed = content.trim();

            if indent == 0 {
                let key = trimmed.strip_suffix(':').ok_or_else(|| MapError::Parse {
                    line,
                    what: format!("expected a `section:` header, found {trimmed:?}"),
                })?;
                section = key.to_string();
                continue;
            }

            if section == "landmarks" {
                let body = trimmed
                    .strip_prefix("- {")
                    .and_then(|s| s.strip_suffix('}'))
                    .ok_or_else(|| MapError::Parse {
                        line,
                        what: format!(
                            "expected `- {{id: ..., x: ..., y: ...}}`, found {trimmed:?}"
                        ),
                    })?;
                let f = flow_fields(line, body)?;
                let name = f.get("id").ok_or_else(|| MapError::Parse {
                    line,
                    what: "landmark has no id".to_string(),
                })?;
                let get = |key: &str| -> Result<f64, MapError> {
                    let raw = f.get(key).ok_or_else(|| MapError::Parse {
                        line,
                        what: format!("landmark {name} has no {key}"),
                    })?;
                    parse_scalar(line, key, raw)
                };
                let (x, y) = (get("x")?, get("y")?);
                let radius = get("radius").unwrap_or(0.0);
                out.landmarks.push(LandmarkEntry {
                    name: name.clone(),
                    x,
                    y,
                    radius,
                });
            } else if section == "robot" && trimmed.starts_with("spawn_pose:") {
                let body = trimmed
                    .split_once('{')
                    .and_then(|(_, rest)| rest.strip_suffix('}'))
                    .ok_or_else(|| MapError::Parse {
                        line,
                        what: format!(
                            "expected `spawn_pose: {{x: .., y: .., theta: ..}}`, found {trimmed:?}"
                        ),
                    })?;
                let f = flow_fields(line, body)?;
                let get = |key: &str| -> Result<f64, MapError> {
                    let raw = f.get(key).ok_or_else(|| MapError::Parse {
                        line,
                        what: format!("spawn_pose has no {key}"),
                    })?;
                    parse_scalar(line, key, raw)
                };
                out.spawn_pose = Some(Se2::new(get("x")?, get("y")?, get("theta")?));
            }
            // Other sections (world geometry) are not needed by localization.
        }
        if out.landmarks.is_empty() {
            return Err(MapError::Empty("the file lists no landmarks"));
        }
        Ok(out)
    }

    /// The landmarks as the model's [`LandmarkMap`]; ids are the file order.
    pub fn landmark_map(&self) -> LandmarkMap {
        let landmarks = self
            .landmarks
            .iter()
            .enumerate()
            .map(|(i, l)| Landmark {
                id: i as u32,
                position: nalgebra::Vector2::new(l.x, l.y),
            })
            .collect();
        // Ids are the enumeration index, so they are unique and the positions were parsed
        // as finite numbers; `LandmarkMap::new` cannot fail here.
        LandmarkMap::new(landmarks).expect("file order gives unique ids and finite positions")
    }

    /// Largest radius in the file, which is what the pillar detector corrects ranges by.
    pub fn nominal_radius(&self) -> f64 {
        self.landmarks
            .iter()
            .map(|l| l.radius)
            .fold(0.0_f64, f64::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "\
# a comment
world:
  name: turtlebot3_localization_room
  floor_size: [6.0, 6.0]

robot:
  model: TurtleBot3 Burger
  spawn_pose: {x: 0.0, y: 0.0, theta: 0.0}

landmarks:
  - {id: landmark_a, x: 1.8, y: 1.2, radius: 0.15, height: 0.6}
  - {id: landmark_b, x: -1.4, y: 2.0, radius: 0.15, height: 0.6}
";

    #[test]
    fn parses_landmarks_and_spawn_pose() {
        let m = MapFile::parse(SAMPLE).unwrap();
        assert_eq!(m.landmarks.len(), 2);
        assert_eq!(m.landmarks[0].name, "landmark_a");
        assert_eq!(m.landmarks[0].x, 1.8);
        assert_eq!(m.landmarks[1].y, 2.0);
        assert_eq!(m.nominal_radius(), 0.15);
        let p = m.spawn_pose.unwrap();
        assert_eq!((p.x(), p.y(), p.theta()), (0.0, 0.0, 0.0));
    }

    #[test]
    fn ids_follow_file_order() {
        let map = MapFile::parse(SAMPLE).unwrap().landmark_map();
        assert_eq!(map.len(), 2);
        assert_eq!(map.get(0).unwrap().position.x, 1.8);
        assert_eq!(map.get(1).unwrap().position.x, -1.4);
    }

    #[test]
    fn a_malformed_landmark_is_an_error_not_a_skipped_line() {
        let bad = "landmarks:\n  - {id: a, x: 1.0}\n";
        assert!(matches!(
            MapFile::parse(bad),
            Err(MapError::Parse { line: 2, .. })
        ));
        let worse = "landmarks:\n  - id: a\n";
        assert!(matches!(
            MapFile::parse(worse),
            Err(MapError::Parse { line: 2, .. })
        ));
    }

    #[test]
    fn a_file_without_landmarks_is_rejected() {
        assert!(matches!(
            MapFile::parse("world:\n  name: x\n"),
            Err(MapError::Empty(_))
        ));
    }

    #[test]
    fn reads_the_simulation_map_in_this_repository() {
        // The file localization actually runs against; keeps the reader honest.
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../sim/webots/config/landmarks.yaml"
        );
        let m = MapFile::load(path).expect("the repository's landmarks.yaml must parse");
        assert_eq!(m.landmarks.len(), 4);
        assert!(m.spawn_pose.is_some());
        assert!(m.nominal_radius() > 0.0);
    }
}

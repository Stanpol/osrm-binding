use serde::{Deserialize, Deserializer, Serialize};

fn deserialize_nodes<'de, D>(deserializer: D) -> Result<Option<Vec<u64>>, D::Error>
where
    D: Deserializer<'de>,
{
    use serde::de::Error;
    
    let value: Option<serde_json::Value> = Option::deserialize(deserializer)?;
    
    match value {
        None => Ok(None),
        Some(serde_json::Value::Array(arr)) => {
            let mut nodes = Vec::new();
            for item in arr {
                match item {
                    serde_json::Value::Number(n) => {
                        if let Some(i) = n.as_u64() {
                            nodes.push(i);
                        } else if let Some(f) = n.as_f64() {
                            nodes.push(f as u64);
                        } else {
                            return Err(D::Error::custom("invalid node id"));
                        }
                    }
                    _ => return Err(D::Error::custom("node must be a number")),
                }
            }
            Ok(Some(nodes))
        }
        _ => Err(D::Error::custom("nodes must be an array")),
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Waypoint {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hint: Option<String>,
    pub location: [f64; 2],
    pub name: String,
    pub distance: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(deserialize_with = "deserialize_nodes")]
    pub nodes: Option<Vec<u64>>,
}

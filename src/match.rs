// match.rs
use crate::point::Point;
use crate::waypoints::Waypoint;
use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Request for map matching GPS traces to the road network
#[derive(Debug, Clone)]
pub struct MatchRequest {
    /// GPS coordinates to match
    pub points: Vec<Point>,
    /// Timestamps for each coordinate (UNIX-like timestamp in seconds)
    pub timestamps: Option<Vec<u32>>,
    /// Standard deviation of GPS precision in meters (use GPS accuracy if available)
    pub radiuses: Option<Vec<Option<f64>>>,
    /// Bearing and range constraints for each coordinate
    pub bearings: Option<Vec<Option<(i16, i16)>>>,
    /// Hints for coordinate snapping (base64 encoded strings)
    pub hints: Option<Vec<Option<String>>>,
    /// Whether to generate hints in response
    pub generate_hints: bool,
    /// Approach constraints for each coordinate
    pub approaches: Option<Vec<Option<String>>>,
    /// Gap handling: "split" (default) or "ignore"
    pub gaps: Option<String>,
    /// Allow track modification for better matching quality
    pub tidy: bool,
    /// Indices of coordinates to treat as waypoints (must include first and last)
    pub waypoints: Option<Vec<usize>>,
    /// Snapping type: "default" or "any"
    pub snapping: Option<String>,
    /// Return turn-by-turn instructions
    pub steps: bool,
    /// Annotations to include (duration, distance, nodes, datasources, weight, speed)
    pub annotations: Option<Vec<String>>,
    /// Geometry format: "polyline", "polyline6", or "geojson"
    pub geometries: Option<String>,
    /// Overview detail level: "simplified", "full", or "false"
    pub overview: Option<String>,
    /// Road types to exclude
    pub exclude: Option<Vec<String>>,
}

/// A matched route segment with confidence score
#[derive(Debug, Deserialize, Serialize)]
pub struct Matching {
    /// The matched route legs
    pub legs: Vec<crate::route::Leg>,
    /// Name of the weight profile
    pub weight_name: String,
    /// Geometry of the matched route
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geometry: Option<Value>,  // Can be String (polyline) or Object (GeoJSON), None when overview=false
    /// Total weight of the matched route
    pub weight: f64,
    /// Total duration in seconds
    pub duration: f64,
    /// Total distance in meters
    pub distance: f64,
    /// Confidence score for this matching (0.0 to 1.0)
    pub confidence: f64,
}

/// Response from a map matching request
#[derive(Debug, Deserialize, Serialize)]
pub struct MatchResponse {
    /// Status code
    pub code: String,
    /// Array of matched route segments
    pub matchings: Vec<Matching>,
    /// Array of tracepoints (matched waypoints, can be null for unmatched points)
    pub tracepoints: Vec<Option<Waypoint>>,
}

use serde::{Deserialize, Serialize};
use crate::point::Point;
use crate::waypoints::Waypoint;

#[derive(Debug, Clone)]
pub struct NearestRequest {
    pub coordinate: Point,
    pub bearings: Option<Vec<Option<(i16, i16)>>>,
    pub radiuses: Option<Vec<Option<f64>>>,
    pub hints: Option<Vec<Option<String>>>,
    pub generate_hints: bool,
    pub number: Option<i32>,
    pub approaches: Option<Vec<Option<String>>>,
    pub snapping: Option<String>,
}

impl NearestRequest {
    pub fn new(coordinate: Point) -> Self {
        Self {
            coordinate,
            bearings: None,
            radiuses: None,
            hints: None,
            generate_hints: true,
            number: Some(1),
            approaches: None,
            snapping: None,
        }
    }
}

pub struct NearestRequestBuilder {
    coordinate: Point,
    bearings: Option<Vec<Option<(i16, i16)>>>,
    radiuses: Option<Vec<Option<f64>>>,
    hints: Option<Vec<Option<String>>>,
    generate_hints: bool,
    number: Option<i32>,
    approaches: Option<Vec<Option<String>>>,
    snapping: Option<String>,
}

impl NearestRequestBuilder {
    pub fn new(coordinate: Point) -> Self {
        Self {
            coordinate,
            bearings: None,
            radiuses: None,
            hints: None,
            generate_hints: true,
            number: Some(1),
            approaches: None,
            snapping: None,
        }
    }

    pub fn bearings(mut self, bearings: Vec<Option<(i16, i16)>>) -> Self {
        self.bearings = Some(bearings);
        self
    }

    pub fn radiuses(mut self, radiuses: Vec<Option<f64>>) -> Self {
        self.radiuses = Some(radiuses);
        self
    }

    pub fn hints(mut self, hints: Vec<Option<String>>) -> Self {
        self.hints = Some(hints);
        self
    }

    pub fn generate_hints(mut self, generate_hints: bool) -> Self {
        self.generate_hints = generate_hints;
        self
    }

    pub fn number(mut self, number: i32) -> Self {
        self.number = Some(number);
        self
    }

    pub fn approaches(mut self, approaches: Vec<Option<String>>) -> Self {
        self.approaches = Some(approaches);
        self
    }

    pub fn snapping(mut self, snapping: String) -> Self {
        self.snapping = Some(snapping);
        self
    }

    pub fn build(self) -> NearestRequest {
        NearestRequest {
            coordinate: self.coordinate,
            bearings: self.bearings,
            radiuses: self.radiuses,
            hints: self.hints,
            generate_hints: self.generate_hints,
            number: self.number,
            approaches: self.approaches,
            snapping: self.snapping,
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct NearestResponse {
    pub code: String,
    pub waypoints: Vec<Waypoint>,
}

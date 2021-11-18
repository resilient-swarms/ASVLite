//! Definition of custom error type thrown by ASVLite-Rust.

use std::fmt;

/// Defining ValueError type. 
#[derive(Debug, Clone, PartialEq)]
pub struct ValueError {
    message: String,
}

impl ValueError {
    /// Constructor.
    pub fn new(message: &str) -> ValueError {
        ValueError{message: String::from(message)}
    }
}

impl fmt::Display for ValueError {
    /// Display error.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

#[test]
fn value_error_new() {
    let error_message = "Unit test for error creation.";
    let test_error = ValueError::new(error_message);
    assert_eq!(format!("{}", test_error), error_message);
}
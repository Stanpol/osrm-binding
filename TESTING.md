# OSRM Bindings - Testing Guide

## Running Tests

### Quick Test (No Data Required)

The tests will automatically skip if OSRM test data is not available:

```bash
cargo test
```

All tests will pass with a message indicating they were skipped:
```
Skipping test: OSRM_TEST_DATA_PATH_MLD environment variable not set
```

### Full Test (With OSRM Data)

To run the full integration tests, you need to:

1. **Prepare OSRM Data**

Download and process map data (e.g., for France):

```bash
# Download OSM data (example: Monaco, small dataset for testing)
wget http://download.geofabrik.de/europe/monaco-latest.osm.pbf

# Extract graph (MLD algorithm)
osrm-extract -p /usr/local/share/osrm/profiles/car.lua monaco-latest.osm.pbf
osrm-partition monaco-latest.osrm
osrm-customize monaco-latest.osrm

# For CH algorithm (optional)
osrm-extract -p /usr/local/share/osrm/profiles/car.lua monaco-latest.osm.pbf
osrm-contract monaco-latest.osrm
```

2. **Set Environment Variable**

Create a `.env` file in the `osrm-binding` directory:

```bash
# .env file
OSRM_TEST_DATA_PATH_MLD=/path/to/monaco-latest.osrm
```

Or export the environment variable:

```bash
export OSRM_TEST_DATA_PATH_MLD=/path/to/monaco-latest.osrm
```

3. **Run Tests**

```bash
cargo test
```

The tests will now execute against the OSRM data.

## Test Structure

The test suite includes:

- **Table Query Test**: Tests distance/duration matrix computation
- **Route Query Test**: Tests routing between two points
- **Simple Route Test**: Tests basic routing functionality
- **Trip Query Test**: Tests traveling salesman problem solving

## Test Data Requirements

The tests expect:
- A processed OSRM dataset (`.osrm` files)
- MLD algorithm data (partition + customize)
- Coverage of test coordinates:
  - Paris: (48.8566, 2.3522)
  - Marseille: (43.2965, 5.3698)
  - Lyon: (45.7640, 4.8357)

For testing with smaller datasets (like Monaco), the tests will be skipped as the coordinates are not in range.

## Continuous Integration

For CI/CD pipelines, you can either:

1. **Skip integration tests** (default behavior without data)
2. **Download test data** in CI setup:
   ```yaml
   before_script:
     - wget http://download.geofabrik.de/europe/monaco-latest.osm.pbf
     - osrm-extract -p /usr/local/share/osrm/profiles/car.lua monaco-latest.osm.pbf
     - osrm-partition monaco-latest.osrm
     - osrm-customize monaco-latest.osrm
     - export OSRM_TEST_DATA_PATH_MLD=monaco-latest.osrm
   ```

## Adding New Tests

When adding new tests that require OSRM data:

```rust
#[test]
fn my_new_test() {
    // Try to load .env file, but don't fail if it doesn't exist
    let _ = dotenvy::dotenv();
    
    let path = match std::env::var("OSRM_TEST_DATA_PATH_MLD") {
        Ok(p) => p,
        Err(_) => {
            eprintln!("Skipping test: OSRM_TEST_DATA_PATH_MLD environment variable not set");
            return;
        }
    };
    
    // Your test code here
    let engine = OsrmEngine::new(&*path, Algorithm::MLD, None)
        .expect("Failed to initialize OSRM engine");
    
    // ... rest of test
}
```

## Troubleshooting

**Tests fail with "Failed to initialize OSRM engine"**
- Check that the OSRM data path is correct
- Verify the data was processed with the correct algorithm (MLD or CH)
- Ensure all required `.osrm.*` files are present

**Tests skip but I have data**
- Check the environment variable is set: `echo $OSRM_TEST_DATA_PATH_MLD`
- Verify the `.env` file is in the correct directory (`osrm-binding/`)
- Try setting the variable explicitly: `OSRM_TEST_DATA_PATH_MLD=/path/to/data.osrm cargo test`

**Test coordinates don't match my region**
- The tests use French coordinates by default
- For other regions, you may need to adjust the test coordinates or use appropriate test data

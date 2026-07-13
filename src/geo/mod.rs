/// Haversine distance in metres (matches C++ `computeHaversineDistance`).
pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const EARTH_RADIUS: f64 = 6_371_000.0;
    const DEG2RAD: f64 = std::f64::consts::PI / 180.0;

    let lat1_rad = lat1 * DEG2RAD;
    let lat2_rad = lat2 * DEG2RAD;
    let diff_lat = (lat2 - lat1) * DEG2RAD;
    let diff_lon = (lon2 - lon1) * DEG2RAD;

    let a = (diff_lat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (diff_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    EARTH_RADIUS * c
}

/// Initial bearing from point 1 to point 2 in radians.
pub fn bearing(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const DEG2RAD: f64 = std::f64::consts::PI / 180.0;
    let lat1_rad = lat1 * DEG2RAD;
    let lat2_rad = lat2 * DEG2RAD;
    let diff_lon = (lon2 - lon1) * DEG2RAD;

    let y = diff_lon.sin() * lat2_rad.cos();
    let x = lat1_rad.cos() * lat2_rad.sin() - lat1_rad.sin() * lat2_rad.cos() * diff_lon.cos();
    y.atan2(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn haversine_same_point_is_zero() {
        assert_relative_eq!(haversine_distance(0.0, 0.0, 0.0, 0.0), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn haversine_known_distance() {
        // ~1 degree latitude ≈ 111 km
        let d = haversine_distance(0.0, 0.0, 1.0, 0.0);
        assert_relative_eq!(d, 111_195.0, epsilon = 1000.0);
    }
}

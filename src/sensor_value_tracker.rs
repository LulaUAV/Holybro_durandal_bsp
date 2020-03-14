/// Implements exponential weighted moving average of sensor readings,
/// including exponentially fading minimum and maximum
pub struct SensorValueTracker {
    /// recent minimum value (not global minimum)
    local_min: f64,
    /// recent maximum value (not global maximum)
    local_max: f64,
    /// exponentially weighted moving average
    average: f64,
    /// weighting factor-- bigger alpha causes faster fade of old values
    alpha: f64,
}

impl SensorValueTracker {
    pub fn new(alpha: f64) -> Self {
        Self {
            local_min: core::f64::NAN,
            local_max: core::f64::NAN,
            average: core::f64::NAN,
            alpha: alpha
        }
    }

    pub fn average(&self) -> f64 {
        self.average
    }

    pub fn range(&self) -> f64 {
        let mut range = self.local_max - self.local_min;
        if range == 0.0 {
            range = 1.0;
        }

        if range < 0.0 { -range }
        else { range }
    }

    pub fn update(&mut self, new_value: f64) -> f64 {
        //seed the EMWA with the initial value
        if self.local_min.is_nan() { self.local_min = new_value; }
        if self.local_max.is_nan() { self.local_max = new_value; }
        if self.average.is_nan() { self.average = new_value; }

        self.average = (self.alpha * new_value) + (1.0 - self.alpha) * self.average;

        // extrema fade toward average
        if new_value > self.local_max { self.local_max = new_value; }
        else if new_value > self.average {
            self.local_max = (self.alpha * new_value) + (1.0 - self.alpha) * self.local_max;
        }
        if new_value < self.local_min { self.local_min = new_value; }
        else if new_value < self.average {
            self.local_min = (self.alpha * new_value) + (1.0 - self.alpha) * self.local_min;
        }

        self.average
    }

}
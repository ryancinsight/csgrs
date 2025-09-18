//! Noise generation algorithms for procedural content
//!
//! This module provides various noise functions including Perlin noise,
//! Simplex noise, and fractal noise combinations for procedural generation.

use super::algorithms::*;

/// Perlin noise generator
#[derive(Debug, Clone)]
pub struct PerlinNoise {
    seed: u64,
    permutation: Vec<usize>,
}

impl PerlinNoise {
    /// Create a new Perlin noise generator
    pub fn new(seed: u64) -> Self {
        let mut permutation = (0..256).collect::<Vec<usize>>();
        let mut rng = seed;

        // Shuffle permutation table
        for i in (1..256).rev() {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            let j = (rng % (i + 1) as u64) as usize;
            permutation.swap(i, j);
        }

        // Duplicate for easier indexing
        permutation.extend_from_slice(&permutation.clone());

        Self { seed, permutation }
    }

    /// Generate 2D Perlin noise
    pub fn noise_2d(&self, x: f64, y: f64) -> f64 {
        // Determine grid cell coordinates
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        // Determine interpolation weights
        let sx = self.fade(x - x0 as f64);
        let sy = self.fade(y - y0 as f64);

        // Interpolate between grid point gradients
        let n0 = self.dot_grid_gradient(x0, y0, x, y);
        let n1 = self.dot_grid_gradient(x1, y0, x, y);
        let ix0 = lerp(n0, n1, sx);

        let n2 = self.dot_grid_gradient(x0, y1, x, y);
        let n3 = self.dot_grid_gradient(x1, y1, x, y);
        let ix1 = lerp(n2, n3, sx);

        lerp(ix0, ix1, sy)
    }

    /// Generate 3D Perlin noise
    pub fn noise_3d(&self, x: f64, y: f64, z: f64) -> f64 {
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let z0 = z.floor() as i32;
        let x1 = x0 + 1;
        let y1 = y0 + 1;
        let z1 = z0 + 1;

        let sx = self.fade(x - x0 as f64);
        let sy = self.fade(y - y0 as f64);
        let sz = self.fade(z - z0 as f64);

        // Trilinear interpolation
        let mut values = [0.0; 8];
        values[0] = self.dot_grid_gradient_3d(x0, y0, z0, x, y, z);
        values[1] = self.dot_grid_gradient_3d(x1, y0, z0, x, y, z);
        values[2] = self.dot_grid_gradient_3d(x0, y1, z0, x, y, z);
        values[3] = self.dot_grid_gradient_3d(x1, y1, z0, x, y, z);
        values[4] = self.dot_grid_gradient_3d(x0, y0, z1, x, y, z);
        values[5] = self.dot_grid_gradient_3d(x1, y0, z1, x, y, z);
        values[6] = self.dot_grid_gradient_3d(x0, y1, z1, x, y, z);
        values[7] = self.dot_grid_gradient_3d(x1, y1, z1, x, y, z);

        trilinear_interpolate(
            values[0], values[1], values[2], values[3],
            values[4], values[5], values[6], values[7],
            sx, sy, sz,
        )
    }

    /// Fade function for Perlin noise
    fn fade(&self, t: f64) -> f64 {
        t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
    }

    /// Calculate gradient at grid point
    fn dot_grid_gradient(&self, ix: i32, iy: i32, x: f64, y: f64) -> f64 {
        let gradient = self.gradient(ix, iy);
        let dx = x - ix as f64;
        let dy = y - iy as f64;
        dx * gradient.0 + dy * gradient.1
    }

    /// Calculate 3D gradient at grid point
    fn dot_grid_gradient_3d(&self, ix: i32, iy: i32, iz: i32, x: f64, y: f64, z: f64) -> f64 {
        let gradient = self.gradient_3d(ix, iy, iz);
        let dx = x - ix as f64;
        let dy = y - iy as f64;
        let dz = z - iz as f64;
        dx * gradient.0 + dy * gradient.1 + dz * gradient.2
    }

    /// Get 2D gradient vector
    fn gradient(&self, x: i32, y: i32) -> (f64, f64) {
        let hash = self.permutation[(self.permutation[(x & 255) as usize] + (y & 255) as usize) & 255];
        match hash & 3 {
            0 => (1.0, 1.0),
            1 => (-1.0, 1.0),
            2 => (1.0, -1.0),
            _ => (-1.0, -1.0),
        }
    }

    /// Get 3D gradient vector
    fn gradient_3d(&self, x: i32, y: i32, z: i32) -> (f64, f64, f64) {
        let hash = self.permutation[(self.permutation[(self.permutation[(x & 255) as usize] + (y & 255) as usize) & 255] + (z & 255) as usize) & 255];
        match hash & 15 {
            0 => (1.0, 1.0, 0.0),
            1 => (-1.0, 1.0, 0.0),
            2 => (1.0, -1.0, 0.0),
            3 => (-1.0, -1.0, 0.0),
            4 => (1.0, 0.0, 1.0),
            5 => (-1.0, 0.0, 1.0),
            6 => (1.0, 0.0, -1.0),
            7 => (-1.0, 0.0, -1.0),
            8 => (0.0, 1.0, 1.0),
            9 => (0.0, -1.0, 1.0),
            10 => (0.0, 1.0, -1.0),
            11 => (0.0, -1.0, -1.0),
            12 => (1.0, 1.0, 0.0),
            13 => (-1.0, 1.0, 0.0),
            14 => (0.0, -1.0, 1.0),
            _ => (0.0, -1.0, -1.0),
        }
    }
}

/// Simplex noise generator (placeholder - full implementation would be complex)
#[derive(Debug, Clone)]
pub struct SimplexNoise {
    seed: u64,
}

impl SimplexNoise {
    pub fn new(seed: u64) -> Self {
        Self { seed }
    }

    /// Generate 2D simplex noise (simplified implementation)
    pub fn noise_2d(&self, x: f64, y: f64) -> f64 {
        // Placeholder - real simplex noise is more complex
        // This is a simplified version for demonstration
        let perlin = PerlinNoise::new(self.seed);
        perlin.noise_2d(x, y) * 0.8 + perlin.noise_2d(x * 2.0, y * 2.0) * 0.2
    }
}

/// Fractal noise combining multiple octaves
#[derive(Debug, Clone)]
pub struct FractalNoise {
    noise: PerlinNoise,
    octaves: usize,
    frequency: f64,
    amplitude: f64,
    lacunarity: f64,
    persistence: f64,
}

impl FractalNoise {
    /// Create a new fractal noise generator
    pub fn new(seed: u64) -> Self {
        Self {
            noise: PerlinNoise::new(seed),
            octaves: 4,
            frequency: 1.0,
            amplitude: 1.0,
            lacunarity: 2.0,
            persistence: 0.5,
        }
    }

    /// Generate fractal noise at 2D point
    pub fn noise_2d(&self, x: f64, y: f64) -> f64 {
        let mut value = 0.0;
        let mut amplitude = self.amplitude;
        let mut frequency = self.frequency;

        for _ in 0..self.octaves {
            value += self.noise.noise_2d(x * frequency, y * frequency) * amplitude;
            amplitude *= self.persistence;
            frequency *= self.lacunarity;
        }

        value
    }

    /// Generate fractal noise at 3D point
    pub fn noise_3d(&self, x: f64, y: f64, z: f64) -> f64 {
        let mut value = 0.0;
        let mut amplitude = self.amplitude;
        let mut frequency = self.frequency;

        for _ in 0..self.octaves {
            value += self.noise.noise_3d(x * frequency, y * frequency, z * frequency) * amplitude;
            amplitude *= self.persistence;
            frequency *= self.lacunarity;
        }

        value
    }
}

/// Ridged multifractal noise for mountain-like terrain
#[derive(Debug, Clone)]
pub struct RidgedNoise {
    noise: FractalNoise,
    offset: f64,
    gain: f64,
}

impl RidgedNoise {
    pub fn new(seed: u64) -> Self {
        Self {
            noise: FractalNoise::new(seed),
            offset: 1.0,
            gain: 2.0,
        }
    }

    pub fn noise_2d(&self, x: f64, y: f64) -> f64 {
        let mut value = 0.0;
        let mut weight = 1.0;

        for octave in 0..self.noise.octaves {
            let frequency = self.noise.frequency * self.noise.lacunarity.powi(octave as i32);
            let amplitude = self.noise.amplitude * self.noise.persistence.powi(octave as i32);

            let signal = self.noise.noise.noise_2d(x * frequency, y * frequency);
            signal = self.offset - signal.abs();
            signal *= signal;
            signal *= weight;

            weight = signal * self.gain;
            weight = clamp(weight, 0.0, 1.0);

            value += signal * amplitude;
        }

        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perlin_noise() {
        let noise = PerlinNoise::new(42);

        let val1 = noise.noise_2d(0.0, 0.0);
        let val2 = noise.noise_2d(1.0, 1.0);
        let val3 = noise.noise_3d(0.0, 0.0, 0.0);

        assert!(val1 >= -1.0 && val1 <= 1.0);
        assert!(val2 >= -1.0 && val2 <= 1.0);
        assert!(val3 >= -1.0 && val3 <= 1.0);

        // Same input should give same output
        assert_eq!(noise.noise_2d(0.5, 0.5), noise.noise_2d(0.5, 0.5));
    }

    #[test]
    fn test_fractal_noise() {
        let noise = FractalNoise::new(123);

        let val1 = noise.noise_2d(0.0, 0.0);
        let val2 = noise.noise_3d(0.0, 0.0, 0.0);

        assert!(val1.is_finite());
        assert!(val2.is_finite());
    }

    #[test]
    fn test_ridged_noise() {
        let noise = RidgedNoise::new(456);

        let val = noise.noise_2d(0.0, 0.0);
        assert!(val.is_finite());
        // Ridged noise should typically be positive due to offset
        assert!(val >= -2.0); // Allow some negative values
    }

    #[test]
    fn test_simplex_noise() {
        let noise = SimplexNoise::new(789);

        let val = noise.noise_2d(0.0, 0.0);
        assert!(val >= -1.0 && val <= 1.0);
    }
}

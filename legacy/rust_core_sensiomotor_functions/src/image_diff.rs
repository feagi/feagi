use ndarray::{Array3, Zip};

pub struct ImageDiff {
    latest: Array3<u8>,
    previous: Array3<u8>,
}

impl ImageDiff {
    pub fn new(image_width: usize, image_height: usize, color_depth: usize) -> Self {
        ImageDiff {
            latest: Array3::zeros((color_depth, image_height, image_width)), // z, y, x order!
            previous: Array3::zeros((color_depth, image_height, image_width)),
        }
    }

    pub fn create_gray_scale(image_width: usize, image_height: usize) -> Self {
        let color_depth: usize = 1;
        Self::new(image_width, image_height, color_depth)
    }

    pub fn create_rgb(image_width: usize, image_height: usize) -> Self {
        let color_depth: usize = 3;
        Self::new(image_width, image_height, color_depth)
    }

    /// Returns the shape of the internal array (color_depth, height, width)
    pub fn shape(&self) -> (usize, usize, usize) {
        let shape = self.latest.shape();
        (shape[0], shape[1], shape[2])
    }

    pub fn get_new_delta_from_new_frame(
        &mut self,
        new_image_frame: &Array3<u8>,
    ) -> Result<Array3<u8>, String> {
        // Shape check for sanity
        if new_image_frame.shape() != self.latest.shape() {
            return Err(format!(
                "Resolution mismatch! Expected {:?}, got {:?}",
                self.latest.shape(),
                new_image_frame.shape()
            ));
        }

        // TODO we will need to use a different comparison algorithm
        let mut diff = Array3::zeros(self.latest.dim());

        // In place subtraction with saturating subtraction to avoid overflow
        Zip::from(&mut diff)
            .and(new_image_frame)
            .and(&self.latest)
            .for_each(|d, &new, &old| {
                // Use saturating_sub to prevent overflow
                *d = new.saturating_sub(old);
            });

        // In place replacement
        self.previous = std::mem::replace(&mut self.latest, new_image_frame.clone());

        Ok(diff)
    }
}

#[cfg(test)]
mod tests {
    use crate::ImageDiff;
    use ndarray::Array3;

    #[test]
    fn test_image_diff_new() {
        let width = 10;
        let height = 8;
        let depth = 3;
        let diff = ImageDiff::new(width, height, depth);

        // Check that the shape is as expected
        let shape = diff.shape();
        assert_eq!(shape, (depth, height, width));
    }

    #[test]
    fn test_get_new_delta_from_new_frame() {
        let width = 2;
        let height = 2;
        let depth = 1;
        let mut diff = ImageDiff::new(width, height, depth);

        // Create a new frame with all zeros
        let zeros = Array3::zeros((depth, height, width));

        // First frame should produce all zeros as diff
        let result = diff.get_new_delta_from_new_frame(&zeros);
        assert!(result.is_ok());
        let delta = result.unwrap();
        assert_eq!(delta.sum(), 0);

        // Create a new frame with some values
        let mut new_frame = Array3::zeros((depth, height, width));
        new_frame[[0, 0, 0]] = 100;
        new_frame[[0, 1, 1]] = 50;

        // Second frame should produce non-zero diff
        let result = diff.get_new_delta_from_new_frame(&new_frame);
        assert!(result.is_ok());
        let delta = result.unwrap();
        assert_eq!(delta[[0, 0, 0]], 100);
        assert_eq!(delta[[0, 1, 1]], 50);

        // Passing a new frame with wrong dimensions should fail
        let wrong_frame = Array3::zeros((depth + 1, height, width));
        let result = diff.get_new_delta_from_new_frame(&wrong_frame);
        assert!(result.is_err());
    }

    #[test]
    fn test_saturating_subtraction() {
        let width = 1;
        let height = 1;
        let depth = 1;
        let mut diff = ImageDiff::new(width, height, depth);

        // Set initial frame with higher values
        let mut first_frame = Array3::zeros((depth, height, width));
        first_frame[[0, 0, 0]] = 100;
        diff.get_new_delta_from_new_frame(&first_frame).unwrap();

        // New frame with lower values
        let mut second_frame = Array3::zeros((depth, height, width));
        second_frame[[0, 0, 0]] = 50;

        // Result should be 0 (not underflow)
        let result = diff.get_new_delta_from_new_frame(&second_frame);
        assert!(result.is_ok());
        let delta = result.unwrap();
        assert_eq!(delta[[0, 0, 0]], 0);
    }
}

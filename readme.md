# WindMouse

WindMouse is a Rust implementation of the WindMouse algorithm, designed to generate realistic mouse movement paths. This library simulates mouse movements with consideration for gravity, wind, and randomness to create more human-like cursor trajectories.

See https://ben.land/post/2021/04/25/windmouse-human-mouse-movement/ for more details on the original algorithm

## Features

- Generate realistic mouse movement paths between two points
- Customizable parameters for fine-tuning movement behavior
- Easy-to-use API with sensible defaults

## Installation

Add this to your `Cargo.toml`:

```toml
[dependencies]
windmouse = "0.1.0"
```

## Usage

```rust
use windmouse::{WindMouse, Coordinate};

fn main() {
    let wind_mouse = WindMouse::new_default();
    let start = Coordinate::new(0.0, 0.0);
    let end = Coordinate::new(100.0, 100.0);
    let points = wind_mouse.generate_points(start, end);

    for point in points {
        println!("x: {}, y: {}, wait: {}ms", point[0], point[1], point[2]);
    }
}
```

## API

### `WindMouse`

The main struct that holds the parameters for the mouse movement algorithm.

#### Methods

- `new(mouse_speed: f32, gravity: f32, wind: f32, min_wait: f32, max_wait: f32, max_step: f32, target_area: f32) -> Result<Self, WindMouseError>`

  Creates a new WindMouse instance with the specified parameters.

- `new_default() -> Self`

  Creates a new WindMouse instance with default values for all parameters.

- `generate_points(&self, start: Coordinate, end: Coordinate) -> Vec<[i32; 3]>`

  Generates a series of points representing the mouse movement path from the start coordinate to the end coordinate.

### `Coordinate`

A struct representing a 2D coordinate with floating-point precision.

#### Methods

- `new(x: f32, y: f32) -> Self`

  Creates a new Coordinate instance.

- `as_i32(&self) -> [i32; 2]`

  Converts the floating-point coordinate to integer values.

## Configuration

You can customize the behavior of the WindMouse algorithm by adjusting the following parameters:

- `gravity`: Influences how strongly the mouse is pulled towards the target.
- `wind`: Determines the amount of randomness in the mouse's path.
- `min_wait`: The minimum time (in milliseconds) to wait between mouse movements.
- `max_wait`: The maximum time (in milliseconds) to wait between mouse movements.
- `max_step`: The maximum distance the mouse can move in a single step.
- `target_area`: The distance from the end point at which the algorithm starts to slow down and become more precise.
- `mouse_speed`: A general speed factor for the mouse movement.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
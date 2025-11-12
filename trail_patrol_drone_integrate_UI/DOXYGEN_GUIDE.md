# Doxygen Documentation Guide

## What Was Configured

Your Doxyfile has been configured with the following settings:

- **PROJECT_NAME**: Trail Patrol Drone Integration UI
- **PROJECT_BRIEF**: ROS2 drone control system with obstacle avoidance and UI integration
- **OUTPUT_DIRECTORY**: docs/ (documentation will be generated here)
- **INPUT**: src/ and include/ directories
- **RECURSIVE**: YES (scans subdirectories)
- **EXTRACT_ALL**: YES (documents all code, even without comments)
- **FILE_PATTERNS**: *.cpp, *.hpp, *.h, *.c
- **GENERATE_LATEX**: YES (for PDF generation)
- **CALL_GRAPH**: YES (shows function call relationships)
- **CALLER_GRAPH**: YES (shows who calls each function)

## How to Generate Documentation

### Generate HTML Documentation

```bash
doxygen Doxyfile
```

This will create documentation in the `docs/` directory.

### View HTML Documentation

Open the generated HTML documentation in your browser:

```bash
firefox docs/html/index.html
# or
google-chrome docs/html/index.html
# or
xdg-open docs/html/index.html
```

### Generate PDF Documentation

After running `doxygen Doxyfile`, compile the LaTeX to PDF:

```bash
cd docs/latex
make
```

This will create `refman.pdf` in the `docs/latex/` directory.

View the PDF:
```bash
xdg-open docs/latex/refman.pdf
```

## Adding Doxygen Comments to Your Code

To get better documentation, add Doxygen-style comments to your code:

### Class Documentation Example

```cpp
/**
 * @brief Navigation controller for autonomous drone flight
 *
 * This class handles waypoint navigation, obstacle detection and avoidance,
 * and publishes velocity commands to control the drone.
 */
class Navigation : public rclcpp::Node {
    // ...
};
```

### Function Documentation Example

```cpp
/**
 * @brief Callback for processing laser scan data
 *
 * Performs obstacle detection at multiple threshold levels (emergency, normal)
 * and initiates appropriate avoidance maneuvers.
 *
 * @param msg Laser scan message containing range data
 */
void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // ...
}
```

### Member Variable Documentation Example

```cpp
/**
 * @brief Current position of the drone in 3D space
 */
Position current_position_;

/**
 * @brief Obstacle detection threshold for normal navigation (meters)
 */
double normal_obstacle_threshold_;
```

## Useful Doxygen Commands

- `@brief` - Short description
- `@param` - Parameter description
- `@return` - Return value description
- `@note` - Important note
- `@warning` - Warning message
- `@see` - Cross-reference to related functions
- `@code` / `@endcode` - Code examples

## Cleaning Up

To remove generated documentation:

```bash
rm -rf docs/
```

## Additional Configuration

If you want to customize further, edit the `Doxyfile` directly. Some useful options:

- `GENERATE_LATEX = NO` - Disable PDF generation if you only want HTML
- `EXTRACT_PRIVATE = NO` - Don't document private members
- `HAVE_DOT = NO` - Disable graphs if graphviz is not installed
- `OPTIMIZE_OUTPUT_FOR_C = YES` - If documenting mostly C code

## Troubleshooting

### LaTeX not found
If PDF generation fails, install LaTeX:
```bash
sudo apt install texlive-full
```

### Graphviz not found
If call graphs don't generate, install graphviz:
```bash
sudo apt install graphviz
```

## Integration with Git

Add to your `.gitignore`:
```
# Doxygen output
docs/
```

Keep the Doxyfile in version control:
```bash
git add Doxyfile DOXYGEN_GUIDE.md
git commit -m "Add Doxygen configuration"
```

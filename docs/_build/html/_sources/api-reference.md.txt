# API Reference

## Robot Control API

### Robot Class

```python
class Robot:
    def __init__(self):
        """Initialize the robot."""
        pass

    def move(self, x: float, y: float, theta: float):
        """Move the robot to the specified position and orientation.
        
        Args:
            x (float): X coordinate
            y (float): Y coordinate
            theta (float): Orientation in radians
        """
        pass

    def stop(self):
        """Stop the robot."""
        pass
```

### Sensor API

```python
class Sensor:
    def __init__(self):
        """Initialize the sensor."""
        pass

    def get_data(self):
        """Get sensor data.
        
        Returns:
            dict: Sensor data
        """
        pass
```

## Evaluation API

### Evaluator Class

```python
class Evaluator:
    def __init__(self):
        """Initialize the evaluator."""
        pass

    def evaluate(self, solution):
        """Evaluate a solution.
        
        Args:
            solution: Solution to evaluate
            
        Returns:
            dict: Evaluation results
        """
        pass
```

## Utility Functions

### Math Utilities

```python
def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi].
    
    Args:
        angle (float): Angle to normalize
        
    Returns:
        float: Normalized angle
    """
    pass
```

### File Utilities

```python
def load_config(path: str) -> dict:
    """Load configuration from file.
    
    Args:
        path (str): Path to config file
        
    Returns:
        dict: Configuration
    """
    pass
``` 
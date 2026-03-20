package frc.robot.statesensors;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * RegionHandler - manages geometric regions loaded from a JSON file.
 * Supports rectangle and circle regions for localization / state sensing.
 * <p>Example JSON:
 * {
 *   "region1": {
 *     "type": "rectangle",
 *     "minX": 3.0, "minY": 2.3, "maxX": 6.3, "maxY": 5.8
 *   },
 *   "region2": {
 *     "type": "circle",
 *     "center": [5, 5],
 *     "radius": 3
 *   }
 * }
 */
public class RegionHandler {

  private static final double EPSILON = 1e-9;

  private final Map<String, Region> regions = new HashMap<>();

  private interface Region {

    boolean contains(double x, double y);

    String debugInfo(double x, double y);
  }

  private static class RectangleRegion implements Region {
    private final double minX;
    private final double minY;
    private final double maxX;
    private final double maxY;

    RectangleRegion(double minX, double minY, double maxX, double maxY) {
      this.minX = minX;
      this.minY = minY;
      this.maxX = maxX;
      this.maxY = maxY;
    }

    @Override
    public boolean contains(double x, double y) {
      return x >= minX - EPSILON && x <= maxX + EPSILON
          && y >= minY - EPSILON && y <= maxY + EPSILON;
    }

    @Override
    public String debugInfo(double x, double y) {
      return String.format(
          "Rectangle[minX=%.4f, minY=%.4f, maxX=%.4f, maxY=%.4f]"
              + " | point=(%.4f, %.4f) | inX=%b inY=%b",
          minX, minY, maxX, maxY, x, y,
          x >= minX - EPSILON && x <= maxX + EPSILON,
          y >= minY - EPSILON && y <= maxY + EPSILON
      );
    }
  }

  private static class CircleRegion implements Region {
    private final double cx;
    private final double cy;
    private final double radius;

    CircleRegion(double cx, double cy, double radius) {
      this.cx = cx;
      this.cy = cy;
      this.radius = radius;
    }

    @Override
    public boolean contains(double x, double y) {
      double dx = x - cx;
      double dy = y - cy;
      double distSq = dx * dx + dy * dy;
      double radiusSq = radius * radius;
      return distSq <= radiusSq + EPSILON;
    }

    @Override
    public String debugInfo(double x, double y) {
      double dx = x - cx;
      double dy = y - cy;
      double dist = Math.sqrt(dx * dx + dy * dy);
      return String.format(
          "Circle[cx=%.4f, cy=%.4f, radius=%.4f] | point=(%.4f, %.4f) | dist=%.6f | inside=%b",
          cx, cy, radius, x, y, dist, contains(x, y)
      );
    }
  }

  /**
   * Constructs a RegionHandler by loading regions from the given file path.
   *
   * @param jsonFilePath the path to the JSON file defining the regions
   */
  public RegionHandler(String jsonFilePath) {
    loadRegionsFromJson(new File(jsonFilePath));
  }

  /**
   * Constructs a RegionHandler by loading regions from the given file.
   *
   * @param path the JSON file defining the regions
   */
  public RegionHandler(File path) {
    loadRegionsFromJson(path);
  }

  /**
   * Parses the JSON file and loads all region definitions into memory.
   *
   * @param path the JSON file to parse
   * @throws RuntimeException if the file cannot be read
   * @throws IllegalArgumentException if a region definition is invalid
   */
  private void loadRegionsFromJson(File path) {
    JsonObject root;
    try (FileReader reader = new FileReader(path)) {
      root = JsonParser.parseReader(reader).getAsJsonObject();
    } catch (IOException e) {
      throw new RuntimeException(
          "Failed to read regions JSON file: " + path.getAbsolutePath(), e);
    }

    for (Map.Entry<String, JsonElement> entry : root.entrySet()) {
      String name = entry.getKey();
      JsonObject obj = entry.getValue().getAsJsonObject();

      if (!obj.has("type")) {
        throw new IllegalArgumentException(
            "Region '" + name + "' is missing a 'type' field.");
      }

      String type = obj.get("type").getAsString().toLowerCase().trim();

      switch (type) {
        case "rectangle" -> {
          double minX = obj.get("minX").getAsDouble();
          double minY = obj.get("minY").getAsDouble();
          double maxX = obj.get("maxX").getAsDouble();
          double maxY = obj.get("maxY").getAsDouble();
          if (minX > maxX || minY > maxY) {
            throw new IllegalArgumentException(
                "Rectangle region '" + name + "' has invalid bounds: min > max.");
          }
          regions.put(name, new RectangleRegion(minX, minY, maxX, maxY));
        }
        case "circle" -> {
          JsonArray center = obj.getAsJsonArray("center");
          if (center.size() != 2) {
            throw new IllegalArgumentException(
                "Circle region '" + name
                    + "' center must have exactly 2 elements [x, y].");
          }
          double cx = center.get(0).getAsDouble();
          double cy = center.get(1).getAsDouble();
          double radius = obj.get("radius").getAsDouble();
          if (radius < 0) {
            throw new IllegalArgumentException(
                "Circle region '" + name + "' has a negative radius.");
          }
          regions.put(name, new CircleRegion(cx, cy, radius));
        }
        default -> throw new IllegalArgumentException(
            "Unknown region type '" + type + "' in region '" + name + "'. "
                + "Expected 'rectangle' or 'circle'.");
      }
    }
  }

  /**
   * Checks if a given point (x, y) is inside a specific named region.
   *
   * @param regionName the name of the region to check
   * @param x the x coordinate in field space
   * @param y the y coordinate in field space
   * @return true if the point lies inside the region
   * @throws IllegalArgumentException if the region name doesn't exist
   */
  public boolean inRegion(String regionName, double x, double y) {
    Region region = regions.get(regionName);
    if (region == null) {
      throw new IllegalArgumentException(
          "No such region: '" + regionName + "'. Available: " + regions.keySet());
    }
    return region.contains(x, y);
  }

  /**
   * Checks if the given pose is inside the specified named region.
   *
   * @param regionName the name of the region to check
   * @param pose the robot pose to test
   * @return true if the pose is inside the region
   * @throws IllegalArgumentException if the region name doesn't exist
   */
  public boolean inRegion(String regionName, Pose2d pose) {
    return inRegion(regionName, pose.getX(), pose.getY());
  }

  /**
   * Returns a list of all region names that contain the given point.
   *
   * @param x the x coordinate
   * @param y the y coordinate
   * @return list of region names containing the point
   */
  public List<String> listCurrentRegions(double x, double y) {
    List<String> inside = new ArrayList<>();
    for (Map.Entry<String, Region> entry : regions.entrySet()) {
      if (entry.getValue().contains(x, y)) {
        inside.add(entry.getKey());
      }
    }
    return inside;
  }

  /**
   * Returns a list of all region names that contain the given pose.
   *
   * @param pose the robot pose to test
   * @return list of region names containing the pose
   */
  public List<String> listCurrentRegions(Pose2d pose) {
    return listCurrentRegions(pose.getX(), pose.getY());
  }

  /**
   * Prints debug info for all regions against a given point.
   * Useful for diagnosing why a point isn't matching a region.
   *
   * @param x the x coordinate
   * @param y the y coordinate
   * @return a formatted debug string
   */
  public String debugPoint(double x, double y) {
    StringBuilder sb = new StringBuilder();
    sb.append(String.format("Debug for point (%.6f, %.6f):%n", x, y));
    for (Map.Entry<String, Region> entry : regions.entrySet()) {
      sb.append(String.format("  [%s] %s%n",
          entry.getKey(), entry.getValue().debugInfo(x, y)));
    }
    return sb.toString();
  }

  /**
   * Prints debug info for all regions against the given pose.
   *
   * @param pose the robot pose to debug
   * @return a formatted debug string
   */
  public String debugPoint(Pose2d pose) {
    return debugPoint(pose.getX(), pose.getY());
  }

  /** Returns all loaded region names. */
  public Set<String> getAllRegionNames() {
    return Collections.unmodifiableSet(regions.keySet());
  }
}

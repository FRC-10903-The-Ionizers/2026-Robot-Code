package frc.robot.stateSensors;

import com.google.gson.*;

import edu.wpi.first.math.geometry.Pose2d;

import java.io.FileReader;
import java.io.IOException;
import java.util.*;
import java.io.File;

/**
 * RegionHandler - manages geometric regions loaded from a JSON file.
 * Supports polygon and circle regions for localization / state sensing.
 *
 * Example JSON:
 * {
 *     "region1": {
 *         "type": "polygon",
 *         "points": [[6.3, 2.3], [6.3, 5.8], [3, 5.8], [3, 2.3]]
 *     },
 *     "region2": {
 *         "type": "circle",
 *         "center": [5, 5],
 *         "radius": 3
 *     }
 * }
 */
public class RegionHandler {

    // -------------------------
    // --- Internal Structure ---
    // -------------------------
    private final Map<String, Region> regions = new HashMap<>();

    private interface Region {
        boolean contains(double x, double y);
    }

    private static class PolygonRegion implements Region {
        private final List<double[]> points;

        public PolygonRegion(List<double[]> points) {
            this.points = points;
        }

        @Override
        public boolean contains(double x, double y) {
            boolean inside = false;
            int n = points.size();

            for (int i = 0, j = n - 1; i < n; j = i++) {
                double xi = points.get(i)[0], yi = points.get(i)[1];
                double xj = points.get(j)[0], yj = points.get(j)[1];

                boolean intersect = ((yi > y) != (yj > y)) &&
                        (x < (xj - xi) * (y - yi) / (yj - yi + 0.0) + xi);
                if (intersect) inside = !inside;
            }

            return inside;
        }
    }

    private static class CircleRegion implements Region {
        private final double cx, cy, radius;

        public CircleRegion(double cx, double cy, double radius) {
            this.cx = cx;
            this.cy = cy;
            this.radius = radius;
        }

        @Override
        public boolean contains(double x, double y) {
            double dx = x - cx;
            double dy = y - cy;
            return dx * dx + dy * dy <= radius * radius;
        }
    }

    public RegionHandler(String jsonFilePath){
        loadRegionsFromJson(jsonFilePath);
    }

    public RegionHandler(File path){
        loadRegionsFromJson(path);
    }

    private void loadRegionsFromJson(String path) {
        JsonObject root;
        try (FileReader reader = new FileReader(path)) {
            root = JsonParser.parseReader(reader).getAsJsonObject();
        } catch (IOException e) {
            throw new RuntimeException("Failed to read regions JSON file: " + path, e);
        }
        loadRegionsFromJsonObject(root);
    }

    private void loadRegionsFromJson(File path) {
        JsonObject root;
        try (FileReader reader = new FileReader(path)) {
            root = JsonParser.parseReader(reader).getAsJsonObject();
        } catch (IOException e) {
            throw new RuntimeException("Failed to read regions JSON file: " + path, e);
        }
        loadRegionsFromJsonObject(root);
    }

    private void loadRegionsFromJsonObject(JsonObject root){
        for (Map.Entry<String, JsonElement> entry : root.entrySet()) {
            String name = entry.getKey();
            JsonObject region = entry.getValue().getAsJsonObject();
            String type = region.get("type").getAsString();
    
            switch (type) {
                case "polygon" -> {
                    List<double[]> points = new ArrayList<>();
                    JsonArray ptsArray = region.getAsJsonArray("points");
                    for (JsonElement ptElem : ptsArray) {
                        JsonArray pt = ptElem.getAsJsonArray();
                        points.add(new double[]{pt.get(0).getAsDouble(), pt.get(1).getAsDouble()});
                    }
                    regions.put(name, new PolygonRegion(points));
                }
                case "circle" -> {
                    JsonArray center = region.getAsJsonArray("center");
                    double cx = center.get(0).getAsDouble();
                    double cy = center.get(1).getAsDouble();
                    double radius = region.get("radius").getAsDouble();
                    regions.put(name, new CircleRegion(cx, cy, radius));
                }
                default -> throw new IllegalArgumentException("Unknown region type: " + type);
            }
        }
    }
    

    // -------------------------
    // --- Public API ---
    // -------------------------

    /**
     * Checks if a given point (x, y) is inside a specific region.
     */
    public boolean inRegion(String regionName, double x, double y) {
        Region region = regions.get(regionName);
        if (region == null) {
            throw new IllegalArgumentException("No such region: " + regionName);
        }
        return region.contains(x, y);
    }

    public boolean inRegion(String regionName, Pose2d pose) {
        Region region = regions.get(regionName);
        if (region == null) {
            throw new IllegalArgumentException("No such region: " + regionName);
        }
        return region.contains(pose.getX(), pose.getY());
    }

    /**
     * Returns a list of all regions that contain the given point.
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

    public List<String> listCurrentRegions(Pose2d pose) {
        List<String> inside = new ArrayList<>();
        for (Map.Entry<String, Region> entry : regions.entrySet()) {
            if (entry.getValue().contains(pose.getX(), pose.getY())) {
                inside.add(entry.getKey());
            }
        }
        return inside;
    }

    public Set<String> getAllRegionNames() {
        return regions.keySet();
    }
}

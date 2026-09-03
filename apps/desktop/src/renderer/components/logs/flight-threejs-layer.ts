/**
 * Three.js custom layer for MapLibre GL — renders flight path at altitude
 * with drop lines to ground. Based on mission-threejs-layer.ts pattern.
 */
import * as THREE from 'three';
import { Line2 } from 'three/examples/jsm/lines/Line2.js';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js';
import maplibregl, { type CustomLayerInterface, type CustomRenderMethodInput } from 'maplibre-gl';

export interface FlightPathPoint {
  lon: number;
  lat: number;
  /** Metres above the takeoff point, NOT AMSL: the layer adds terrain itself. */
  alt: number;
}

export interface FlightPathLayerData {
  points: FlightPathPoint[];
  /** Ground elevation MSL at the takeoff point. Added to all altitudes so path sits above terrain. */
  groundElevation?: number;
  terrainExaggeration?: number;

  /** Per-segment hex color (length = points.length - 1). If omitted, uses default amber. */
  segmentColors?: string[];
}

export interface FlightPathThreeJsLayer {
  layer: CustomLayerInterface;
  updateData: (data: FlightPathLayerData) => void;
  dispose: () => void;
}

/**
 * Line width in SCREEN PIXELS. A world-space ribbon or tube has to guess a
 * thickness in metres, which reads as a hairline on a long flight and as a
 * sausage on a short one; a screen-space line is the same weight at any zoom.
 */
const PATH_WIDTH_PX = 3.5;
const DEFAULT_PATH_HEX = '#f59e0b'; // amber
const DROP_LINE_COLOR = 0xf59e0b; // amber, matching path
const DROP_LINE_OPACITY = 0.55;
/** Drop lines are spaced to land ~40 across the track whatever its length. */
const DROP_LINE_TARGET = 40;
/** Drop lines start once the track is clear of the ground by this much. */
const DROP_LINE_MIN_HEIGHT_M = 0.5;

export function createFlightPathThreeJsLayer(): FlightPathThreeJsLayer {
  let map: maplibregl.Map | null = null;
  let renderer: THREE.WebGLRenderer | null = null;
  const camera = new THREE.Camera();
  const scene = new THREE.Scene();

  let modelTransform = { translateX: 0, translateY: 0, translateZ: 0, scale: 1 };
  const rotationX = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(1, 0, 0), Math.PI / 2);

  let pathLine: Line2 | null = null;
  let pathGeometry: LineGeometry | null = null;
  let pathMaterial: LineMaterial | null = null;
  let dropLinesObj: THREE.LineSegments | null = null;

  function clearScene() {
    if (pathLine) {
      scene.remove(pathLine);
      pathGeometry?.dispose();
      pathMaterial?.dispose();
      pathLine = null;
      pathGeometry = null;
      pathMaterial = null;
    }
    if (dropLinesObj) {
      scene.remove(dropLinesObj);
      dropLinesObj.geometry.dispose();
      (dropLinesObj.material as THREE.Material).dispose();
      dropLinesObj = null;
    }
  }

  function rebuildScene(data: FlightPathLayerData) {
    clearScene();
    const { points, groundElevation = 0, segmentColors } = data;
    if (points.length < 2) return;

    // Reference point = centroid at ground elevation
    let sumLon = 0, sumLat = 0;
    for (const p of points) { sumLon += p.lon; sumLat += p.lat; }
    const refLon = sumLon / points.length;
    const refLat = sumLat / points.length;

    const refMc = maplibregl.MercatorCoordinate.fromLngLat([refLon, refLat], 0);
    const s = refMc.meterInMercatorCoordinateUnits();
    modelTransform = { translateX: refMc.x, translateY: refMc.y, translateZ: refMc.z, scale: s };

    // Convert to local coords (Y-up: X=east, Y=alt, Z=south)
    // Use groundElevation (MSL) + AGL altitude directly in meters
    // The model transform (scale = meterInMercatorCoordinateUnits) handles conversion
    const local = points.map(p => {
      const mc = maplibregl.MercatorCoordinate.fromLngLat([p.lon, p.lat], 0);
      return {
        x: (mc.x - refMc.x) / s,
        y: groundElevation + p.alt,
        z: (mc.y - refMc.y) / s,
        groundY: groundElevation,
      };
    });

    // One screen-space polyline through every point. Width is in pixels, so it
    // looks like a line at any zoom instead of a hairline or a solid tube.
    {
      const positions: number[] = [];
      const colors: number[] = [];
      const colorCache = new Map<string, THREE.Color>();
      const segColor = (i: number): THREE.Color => {
        const hex = segmentColors?.[Math.min(i, (segmentColors?.length ?? 1) - 1)] ?? DEFAULT_PATH_HEX;
        let c = colorCache.get(hex);
        if (!c) { c = new THREE.Color(hex); colorCache.set(hex, c); }
        return c;
      };

      for (let i = 0; i < local.length; i++) {
        const p = local[i]!;
        positions.push(p.x, p.y, p.z);
        // segmentColors is per segment; a vertex takes the colour of the
        // segment leaving it, so a mode change blends across one segment.
        const c = segColor(Math.min(i, local.length - 2));
        colors.push(c.r, c.g, c.b);
      }

      pathGeometry = new LineGeometry();
      pathGeometry.setPositions(positions);
      pathGeometry.setColors(colors);

      pathMaterial = new LineMaterial({
        linewidth: PATH_WIDTH_PX,
        vertexColors: true,
        // Pixel widths, not metres. resolution is refreshed every frame in
        // render() because the canvas can resize under us.
        worldUnits: false,
        dashed: false,
      });

      pathLine = new Line2(pathGeometry, pathMaterial);
      pathLine.frustumCulled = false;
      scene.add(pathLine);
    }

    // Drop lines — every N points
    {
      const positions: number[] = [];
      const step = Math.max(1, Math.round(local.length / DROP_LINE_TARGET));
      for (let i = 0; i < local.length; i += step) {
        const p = local[i]!;
        if (p.y > p.groundY + DROP_LINE_MIN_HEIGHT_M) {
          positions.push(p.x, p.y, p.z);
          positions.push(p.x, p.groundY, p.z);
        }
      }
      // Always include last point
      const last = local[local.length - 1]!;
      if (last.y > last.groundY + DROP_LINE_MIN_HEIGHT_M) {
        positions.push(last.x, last.y, last.z);
        positions.push(last.x, last.groundY, last.z);
      }

      if (positions.length > 0) {
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));

        const mat = new THREE.LineDashedMaterial({
          color: DROP_LINE_COLOR,
          transparent: true,
          opacity: DROP_LINE_OPACITY,
          dashSize: 5,
          gapSize: 5,
        });

        dropLinesObj = new THREE.LineSegments(geom, mat);
        dropLinesObj.computeLineDistances();
        dropLinesObj.frustumCulled = false;
        scene.add(dropLinesObj);
      }
    }
  }

  const layer: CustomLayerInterface = {
    id: 'flight-path-threejs',
    type: 'custom',
    renderingMode: '3d',

    onAdd(m, gl) {
      map = m;
      renderer = new THREE.WebGLRenderer({ canvas: m.getCanvas(), context: gl, antialias: true });
      renderer.autoClear = false;
    },

    render(_gl, args: CustomRenderMethodInput) {
      if (!renderer || !map) return;

      const { translateX, translateY, translateZ, scale: sc } = modelTransform;
      const l = new THREE.Matrix4()
        .makeTranslation(translateX, translateY, translateZ)
        .scale(new THREE.Vector3(sc, -sc, sc))
        .multiply(rotationX);

      const m = new THREE.Matrix4().fromArray(args.defaultProjectionData.mainMatrix as number[]);
      camera.projectionMatrix = m.multiply(l);

      if (pathMaterial) {
        // CSS pixels, not drawing-buffer pixels: LineMaterial divides the width
        // by resolution, so passing device pixels halves the line on a retina
        // display. clientWidth is 0 in a detached canvas, hence the fallback.
        const canvas = map.getCanvas();
        pathMaterial.resolution.set(
          canvas.clientWidth || canvas.width,
          canvas.clientHeight || canvas.height,
        );
      }

      renderer.resetState();
      renderer.render(scene, camera);
    },
  };

  return {
    layer,
    updateData(data: FlightPathLayerData) {
      rebuildScene(data);
      map?.triggerRepaint();
    },
    dispose() {
      clearScene();
    },
  };
}

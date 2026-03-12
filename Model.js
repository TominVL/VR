"use strict";

/* ─────────────────────────────────────────
   VIRICH CYCLIC SURFACE  –  geometry build
───────────────────────────────────────── */

function f_func(v, a, b) {
    return (a * b) / Math.sqrt(a*a*Math.sin(v)*Math.sin(v) + b*b*Math.cos(v)*Math.cos(v));
}

function parametric(u, v, a, b, c, d) {
    let fv = f_func(v, a, b);
    let x = 0.5 * (fv*(1+Math.cos(u)) + (d*d - c*c)*(1-Math.cos(u))/fv) * Math.cos(v);
    let y = 0.5 * (fv*(1+Math.cos(u)) + (d*d - c*c)*(1-Math.cos(u))/fv) * Math.sin(v);
    let z = 0.5 * (fv - (d*d - c*c)/fv) * Math.sin(u);
    let s = 0.6;
    return [x*s, y*s, z*s];
}

function calcNormal(u, v, a, b, c, d) {
    let eps = 0.01;
    let p  = parametric(u, v, a, b, c, d);
    let pu = parametric(u + eps, v, a, b, c, d);
    let pv = parametric(u, v + eps, a, b, c, d);
    let du = [pu[0]-p[0], pu[1]-p[1], pu[2]-p[2]];
    let dv = [pv[0]-p[0], pv[1]-p[1], pv[2]-p[2]];
    let nx = du[1]*dv[2] - du[2]*dv[1];
    let ny = du[2]*dv[0] - du[0]*dv[2];
    let nz = du[0]*dv[1] - du[1]*dv[0];
    let L  = Math.sqrt(nx*nx + ny*ny + nz*nz) || 1;
    return [nx/L, ny/L, nz/L];
}

/**
 * Build surface buffers.
 * Returns { verts, norms, idx, wires } as typed arrays.
 */
function buildSurface(U, V) {
    let a = 2, b = 1, c = 0.5, d = 1.2;
    let verts = [], norms = [], idx = [], wires = [];

    for (let j = 0; j <= V; j++) {
        let v = 2 * Math.PI * j / V;
        for (let i = 0; i <= U; i++) {
            let u = 2 * Math.PI * i / U;
            let p = parametric(u, v, a, b, c, d);
            let n = calcNormal(u, v, a, b, c, d);
            verts.push(...p);
            norms.push(...n);
        }
    }

    let row = U + 1;
    for (let j = 0; j < V; j++) {
        for (let i = 0; i < U; i++) {
            let p0 = j*row + i, p1 = p0+1, p2 = p0+row, p3 = p2+1;
            idx.push(p0, p2, p1,  p1, p2, p3);
        }
    }

    // wireframe edges (along U and V)
    for (let j = 0; j <= V; j++)
        for (let i = 0; i < U; i++)
            wires.push(j*row + i, j*row + i + 1);
    for (let i = 0; i <= U; i++)
        for (let j = 0; j < V; j++)
            wires.push(j*row + i, (j+1)*row + i);

    return {
        verts: new Float32Array(verts),
        norms: new Float32Array(norms),
        idx:   new Uint16Array(idx),
        wires: new Uint16Array(wires)
    };
}

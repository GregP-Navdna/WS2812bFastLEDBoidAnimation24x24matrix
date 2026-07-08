/**
 * vec2 - 2D vector class, ported from vec2.h
 * Mirrors the C++ vec2<float> (PVector) template class
 */
export class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  copy() {
    return new Vec2(this.x, this.y);
  }

  set(x, y) {
    this.x = x;
    this.y = y;
    return this;
  }

  add(v) {
    if (v instanceof Vec2) {
      this.x += v.x;
      this.y += v.y;
    } else {
      this.x += v;
      this.y += v;
    }
    return this;
  }

  sub(v) {
    if (v instanceof Vec2) {
      this.x -= v.x;
      this.y -= v.y;
    } else {
      this.x -= v;
      this.y -= v;
    }
    return this;
  }

  mult(s) {
    this.x *= s;
    this.y *= s;
    return this;
  }

  div(s) {
    if (s !== 0) {
      this.x /= s;
      this.y /= s;
    }
    return this;
  }

  mag() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  }

  magSq() {
    return this.x * this.x + this.y * this.y;
  }

  normalize() {
    const len = this.mag();
    if (len === 0) return this;
    this.mult(1.0 / len);
    return this;
  }

  limit(max) {
    if (this.magSq() > max * max) {
      this.normalize();
      this.mult(max);
    }
    return this;
  }

  dist(v) {
    const dx = v.x - this.x;
    const dy = v.y - this.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  ortho() {
    return new Vec2(this.y, -this.x);
  }

  rotate(deg) {
    const theta = (deg / 180.0) * Math.PI;
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    const tx = this.x * c - this.y * s;
    const ty = this.x * s + this.y * c;
    this.x = tx;
    this.y = ty;
    return this;
  }

  rotateAroundPoint(cx, cy, deg) {
    const theta = (deg / 180.0) * Math.PI;
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    this.x -= cx;
    this.y -= cy;
    const tx = this.x * c - this.y * s;
    const ty = this.x * s + this.y * c;
    this.x = tx + cx;
    this.y = ty + cy;
    return this;
  }

  makePositive() {
    if (this.x < 0) this.x = -this.x;
    if (this.y < 0) this.y = -this.y;
    return this;
  }

  static dot(v1, v2) {
    return v1.x * v2.x + v1.y * v2.y;
  }

  static cross(v1, v2) {
    return v1.x * v2.y - v1.y * v2.x;
  }

  static sub(v1, v2) {
    return new Vec2(v1.x - v2.x, v1.y - v2.y);
  }

  static add(v1, v2) {
    return new Vec2(v1.x + v2.x, v1.y + v2.y);
  }

  static mult(v, s) {
    return new Vec2(v.x * s, v.y * s);
  }
}

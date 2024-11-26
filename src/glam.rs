use crate::impl_Interpolate;
use crate::Interpolate;

use glam::{DQuat, DVec2, DVec3, DVec4, Quat, Vec2, Vec3, Vec3A, Vec4};

impl_Interpolate!(f32, Vec2, std::f32::consts::PI);
impl_Interpolate!(f32, Vec3, std::f32::consts::PI);
impl_Interpolate!(f32, Vec3A, std::f32::consts::PI);
impl_Interpolate!(f32, Vec4, std::f32::consts::PI);

impl_Interpolate!(f64, DVec2, std::f64::consts::PI);
impl_Interpolate!(f64, DVec3, std::f64::consts::PI);
impl_Interpolate!(f64, DVec4, std::f64::consts::PI);

impl Interpolate<f32> for Quat {
  fn step(t: f32, threshold: f32, a: Self, b: Self) -> Self {
    if t < threshold {
      a
    } else {
      b
    }
  }

  fn cosine(t: f32, a: Self, b: Self) -> Self {
    let cos_nt = (1. - (t * std::f32::consts::PI).cos()) * 0.5;
    <Self as Interpolate<f32>>::lerp(cos_nt, a, b)
  }

  fn lerp(t: f32, a: Self, b: Self) -> Self {
    a.slerp(b, t)
  }

  fn cubic_hermite(t: f32, x: (f32, Self), a: (f32, Self), b: (f32, Self), y: (f32, Self)) -> Self {
    unimplemented!("Need to figure this out for quaternions")
  }

  fn quadratic_bezier(t: f32, a: Self, u: Self, b: Self) -> Self {
    // ⚠️ UNTESTED ⚠️
    let q1 = a.slerp(u, t);
    let q2 = u.slerp(b, t);

    q1.slerp(q2, t)
  }

  fn cubic_bezier(t: f32, a: Self, u: Self, v: Self, b: Self) -> Self {
    // ⚠️ UNTESTED ⚠️
    let q1 = a.slerp(u, t);
    let q2 = u.slerp(v, t);
    let q3 = v.slerp(b, t);

    let x = q1.slerp(q2, t);
    let y = q2.slerp(q3, t);

    x.slerp(y, t)
  }

  fn cubic_bezier_mirrored(t: f32, a: Self, u: Self, v: Self, b: Self) -> Self {
    <Self as Interpolate<f32>>::cubic_bezier(t, a, u, b + b - v, b)
  }
}

impl Interpolate<f64> for DQuat {
  fn step(t: f64, threshold: f64, a: Self, b: Self) -> Self {
    if t < threshold {
      a
    } else {
      b
    }
  }

  fn cosine(t: f64, a: Self, b: Self) -> Self {
    let cos_nt = (1. - (t * std::f64::consts::PI).cos()) * 0.5;
    <Self as Interpolate<f64>>::lerp(cos_nt, a, b)
  }

  fn lerp(t: f64, a: Self, b: Self) -> Self {
    a.slerp(b, t)
  }

  fn cubic_hermite(t: f64, x: (f64, Self), a: (f64, Self), b: (f64, Self), y: (f64, Self)) -> Self {
    unimplemented!("Need to figure this out for quaternions")
  }

  fn quadratic_bezier(t: f64, a: Self, u: Self, b: Self) -> Self {
    // ⚠️ UNTESTED ⚠️
    let q1 = a.slerp(u, t);
    let q2 = u.slerp(b, t);

    q1.slerp(q2, t)
  }

  fn cubic_bezier(t: f64, a: Self, u: Self, v: Self, b: Self) -> Self {
    // ⚠️ UNTESTED ⚠️
    let q1 = a.slerp(u, t);
    let q2 = u.slerp(v, t);
    let q3 = v.slerp(b, t);

    let x = q1.slerp(q2, t);
    let y = q2.slerp(q3, t);

    x.slerp(y, t)
  }

  fn cubic_bezier_mirrored(t: f64, a: Self, u: Self, v: Self, b: Self) -> Self {
    <Self as Interpolate<f64>>::cubic_bezier(t, a, u, b + b - v, b)
  }
}

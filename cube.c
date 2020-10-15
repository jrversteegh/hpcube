/* 
   cube - Small demo that displays a rotating cube

   Copyright (C) 2007 Jaap Versteegh

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  
*/

#include <hpgcc49.h>
#include <ggl.h>

#undef LCD_W
#define LCD_W 132

typedef struct {
  double x;
  double y;
  double z;
} Point;

typedef struct {
  Point* b;
  Point* e;
} Line;

typedef struct PointItem {
  Point p;
  struct PointItem* next;
} PointItem;

typedef struct LineItem {
  Line l;
  struct LineItem* next;
} LineItem; 

typedef struct Pixel {
  double u;
  double v;
} Pixel;
 
typedef struct Object {
  PointItem* points;
  LineItem* lines; 
} Object;

typedef struct Camera {
  Point location;
  Point target;
  double FOV;
  Object* obj;
  
  gglsurface* srf;
  gglsurface srfs[2];

  Point d;
  Point u;
  Point v;
} Camera;

Object* object_new(void)
{
  Object* object = (Object*) calloc(1, sizeof(Object));
  return object;
}

void object_delete(Object* obj)
{
  if (obj == NULL) {
    return;
  }
  LineItem* li = obj->lines;
  while (li != NULL) {
    LineItem* oldli = li;
    li = li->next;
    free(oldli);
  }

  PointItem* pi = obj->points;
  while (pi != NULL) {
    PointItem* oldpi = pi;
    pi = pi->next;
    free(oldpi);
  }
  free(obj);
}

PointItem* new_point(double x, double y, double z)
{
  PointItem* pi = (PointItem*) calloc(1, sizeof(PointItem));
  pi->p.x = x;
  pi->p.y = y;
  pi->p.z = z;
  return pi;
}

Point* get_point(Object* obj, double x, double y, double z)
{
  PointItem* pc = obj->points;
  if (pc == NULL) {
    obj->points = new_point(x, y, z);
    return &obj->points->p;
  } else {
    for (;;) {
      if (pc->p.x == x && pc->p.y == y && pc->p.z == z) {
        return &pc->p;
      }
      if (pc->next != NULL) {
        pc = pc->next;
      } else {
        break;
      }
    }
    pc->next = new_point(x, y, z);
    return &pc->next->p;
  }
}

void add_line(Object* obj, 
    double x1, double y1, double z1, 
    double x2, double y2, double z2)
{
  LineItem* li = (LineItem*) calloc(1, sizeof(LineItem));
  LineItem* lc = obj->lines;
  if (lc == NULL) {
    obj->lines = li;
  } else {
    while (lc->next != NULL) {
      lc = lc->next;
    }
    lc->next = li;
  }
  li->l.b = get_point(obj, x1, y1, z1);
  li->l.e = get_point(obj, x2, y2, z2);
}

Object* unit_cube(void)
{
  Object* cube = object_new();
  add_line(cube, -0.5, -0.5, -0.5, -0.5, -0.5, 0.5);
  add_line(cube, -0.5, -0.5, 0.5, -0.5, 0.5, 0.5);
  add_line(cube, -0.5, 0.5, 0.5, -0.5, 0.5, -0.5);
  add_line(cube, -0.5, 0.5, -0.5, -0.5, -0.5, -0.5);
  add_line(cube, 0.5, -0.5, -0.5, 0.5, -0.5, 0.5);
  add_line(cube, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5);
  add_line(cube, 0.5, 0.5, 0.5, 0.5, 0.5, -0.5);
  add_line(cube, 0.5, 0.5, -0.5, 0.5, -0.5, -0.5);
  add_line(cube, -0.5, -0.5, -0.5, 0.5, -0.5, -0.5);
  add_line(cube, -0.5, -0.5, 0.5, 0.5, -0.5, 0.5);
  add_line(cube, -0.5, 0.5, -0.5, 0.5, 0.5, -0.5);
  add_line(cube, -0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
  return cube;
}

void point_normalize(Point* point)
{
  double length_divisor = sqrt(
      1 / (point->x * point->x + point->y * point->y + point->z * point->z));
  point->x = point->x * length_divisor;
  point->y = point->y * length_divisor;
  point->z = point->z * length_divisor;
}

void point_assign(Point* point, Point* source)
{
  point->x = source->x;
  point->y = source->y;
  point->z = source->z;
}

void point_add(Point* point, Point* vec1, Point* vec2)
{
  point->x = vec1->x + vec2->x;
  point->y = vec1->y + vec2->y;
  point->z = vec1->z + vec2->z;
}

void point_sub(Point* point, Point* vec1, Point* vec2)
{
  point->x = vec1->x - vec2->x;
  point->y = vec1->y - vec2->y;
  point->z = vec1->z - vec2->z;
}

void point_mul(Point* point, Point* source, double multiplier)
{
  point->x = source->x * multiplier;
  point->y = source->y * multiplier;
  point->z = source->z * multiplier;
}

double point_dot_product(Point* vec1, Point* vec2)
{
  return vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
}

void point_cross_product(Point* point, Point* vec1, Point* vec2)
{
  point->x = vec1->y * vec2->z - vec1->z * vec2->y;
  point->y = vec1->z * vec2->x - vec1->x * vec2->z;
  point->z = vec1->x * vec2->y - vec1->y * vec2->x;
}

void point_rotate(Point* p, Point* base, Point* axis, double angle)
{
  double cs = cos(angle);
  double sn = sin(angle);
  Point point_based; point_sub(&point_based, p, base);
  Point axis_tangent; point_mul(&axis_tangent, axis, 
      point_dot_product(axis, &point_based));
  Point axis_perp; point_sub(&axis_perp, &point_based, &axis_tangent);
  Point axis_cross; point_cross_product(&axis_cross, axis, &axis_perp);
  point_mul(&axis_perp, &axis_perp, cs - 1);
  point_mul(&axis_cross, &axis_cross, sn);
  point_add(p, p, &axis_perp);
  point_add(p, p, &axis_cross);
}

void object_rotate(Object* obj, Point* base, Point* axis, double angle)
{
  PointItem* pi = obj->points;
  while (pi != NULL) {
    Point* p = &pi->p;
    point_rotate(p, base, axis, angle);
    pi = pi->next;
  }
}

int camera_xyz_to_uv(Camera* cam,  Pixel* pi, Point* point)
{
  static int w_div_2 = LCD_W >> 1;
  static int h_div_2 = LCD_H >> 1;
  Point sight;
  point_sub(&sight, point, &cam->location);
  double dist = point_dot_product(&sight, &cam->d);
  if (dist <= 0 ) {
    return FALSE;
  }
  point_mul(&sight, &sight, 1 / dist); 
  Point uv_vec;
  point_sub(&uv_vec, &sight, &cam->d);
  pi->u = w_div_2 + round(point_dot_product(&uv_vec, &cam->u) * h_div_2);
  pi->v = h_div_2 + round(point_dot_product(&uv_vec, &cam->v) * h_div_2);
  return TRUE;
}

void camera_update(Camera* cam)
{
  Point z_up;
  /* Up on the screen is up in three dimensional space: the camera doesn't 'roll' */
  z_up.x = 0;
  z_up.y = 0;
  z_up.z = 1;
  
  /* Update camera internal variables */
  point_sub(&cam->d, &cam->target, &cam->location);
  point_normalize(&cam->d);
  point_cross_product(&cam->u, &cam->d, &z_up);
  point_normalize(&cam->u);
  point_mul(&cam->u, &cam->u, 1 / cam->FOV);
  point_cross_product(&cam->v, &cam->u, &cam->d);
}

Camera* camera_new(double lx, double ly, double lz, 
    double tx, double ty, double tz, double FOV)
{
  Camera* cam = (Camera*) calloc(1, sizeof(Camera));
  cam->srf = &cam->srfs[0];
  cam->location.x = lx;
  cam->location.y = ly;
  cam->location.z = lz;
  cam->target.x = tx;
  cam->target.y = ty;
  cam->target.z = tz;
  cam->FOV = FOV;
  camera_update(cam);
  ggl_initscr(&cam->srfs[0]);
  ggl_initscr(&cam->srfs[1]);
  ggl_setmode(cam->srfs[0].addr);
  ggl_setmode(cam->srfs[1].addr);
  return cam;
}

void camera_delete(Camera* cam)
{
  ggl_freescr(&cam->srfs[0]);
  ggl_freescr(&cam->srfs[1]);
  free(cam);
}

void camera_flip_buffer(Camera* cam)
{
  /* The current buffer should've finished drawing. Display it now */
  ggl_show(cam->srf->addr);
  /* Then flip for the next redraw */
  cam->srf = cam->srf == &cam->srfs[0] ?  &cam->srfs[1] : &cam->srfs[0];
}

void do_draw(Camera* cam)
{
  camera_flip_buffer(cam);
  ggl_rect(cam->srf, 0, 0, LCD_W - 1, LCD_H - 1, ggl_mkcolor(0));
  Object* obj = cam->obj;
  if (obj == NULL) {
    return;
  }
  ggl_initaline();
  Pixel pixel1;
  Pixel pixel2;
  LineItem* li = obj->lines;
  while (li != NULL) {
    if (camera_xyz_to_uv(cam, &pixel1, li->l.b)
       && camera_xyz_to_uv(cam, &pixel2, li->l.e)) {
      ggl_aline(cam->srf, pixel1.u, pixel1.v, pixel2.u, pixel2.v);
    }
    li = li->next;
  }
  ggl_endaline();
}

int
main (int argc, char **argv)
{
  int* LCDState = (int*) malloc(STATEBUFSIZE);
  ggl_save(LCDState);

  Object* cube = unit_cube(); 
  Camera* camera = camera_new(-3, 0, 0, 0, 0, 0, 0.4);
  camera->obj = cube;
  Point rotate_axis; rotate_axis.x = 0; rotate_axis.y = 0; rotate_axis.z = 1;
  Point x_axis; x_axis.x = 1; x_axis.y = 0; x_axis.z = 0;
  Point y_axis; y_axis.x = 0; y_axis.y = 1; y_axis.z = 0;
  Point z_axis; z_axis.x = 0; z_axis.y = 0; z_axis.z = 1;
  Point rotate_base; rotate_base.x = 0; rotate_base.y = 0; rotate_base.z = 0;
  double rot_speed = 0.05;
  keymatrix keys;
  do {
    keyb_getmatrix(&keys);
    if (keys.loword & KB_MASK32(KB_ADD)) { rot_speed += 0.01; }
    if (keys.loword & KB_MASK32(KB_SUB)) { rot_speed -= 0.01; }
    if (keys.loword & KB_MASK32(KB_MUL)) { 
      camera->location.x += 0.1;
      camera_update(camera);
    };
    if (keys.loword & KB_MASK32(KB_DIV)) { 
      camera->location.x -= 0.1;
      camera_update(camera);
    };
    if (keys.hiword & KB_MASK32(KB_UP)) { 
      point_rotate(&rotate_axis, &rotate_base, &y_axis, 0.1);
    }
    if (keys.hiword & KB_MASK32(KB_DN)) { 
      point_rotate(&rotate_axis, &rotate_base, &y_axis, -0.1);
    }
    if (keys.hiword & KB_MASK32(KB_RT)) { 
      point_rotate(&rotate_axis, &rotate_base, &z_axis, 0.1);
    }
    if (keys.hiword & KB_MASK32(KB_LF)) { 
      point_rotate(&rotate_axis, &rotate_base, &z_axis, -0.1);
    }
    do_draw(camera);
    object_rotate(cube, &rotate_base, &rotate_axis, rot_speed);
  } while (!(keys.hiword & KB_MASK32(KB_ON)));
  ggl_restore(LCDState);
  free(LCDState);
  camera_delete(camera);
  object_delete(cube);
  return 0;
}


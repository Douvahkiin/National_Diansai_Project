#include "pr.h"

struct _pr pr1;
struct _pr pr2;
struct _pr pr3;
struct _pr pr4;

void pr_init(float32 d1, float32 d2, float32 d3, float32 n1, float32 n2, float32 n3, struct _pr* prx) {
  prx->d1 = d1;
  prx->d2 = d2;
  prx->d3 = d3;
  prx->n1 = n1;
  prx->n2 = n2;
  prx->n3 = n3;
  prx->yk_1 = 0;
  prx->yk_2 = 0;
  prx->uk_1 = 0;
  prx->uk_2 = 0;
}

float32 pr_run(float32 uk, struct _pr* prx) {
  float32 yk = (-prx->d2 * prx->yk_1 - prx->d3 * prx->yk_2 + prx->n1 * uk + prx->n2 * prx->uk_1 + prx->n3 * prx->uk_2) / prx->d1;
  prx->yk_2 = prx->yk_1;
  prx->yk_1 = yk;
  prx->uk_2 = prx->uk_1;
  prx->uk_1 = uk;
  return yk;
}

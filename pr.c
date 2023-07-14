#include "pr.h"

struct _pr pr;

void pr_init(float32 d1, float32 d2, float32 d3, float32 n1, float32 n2, float32 n3) {
  pr.d1 = d1;
  pr.d2 = d2;
  pr.d3 = d3;
  pr.n1 = n1;
  pr.n2 = n2;
  pr.n3 = n3;
  pr.yk_1 = 0;
  pr.yk_2 = 0;
  pr.uk_1 = 0;
  pr.uk_2 = 0;
}

float32 pr_run(float32 uk) {
  float32 yk = (-pr.d2 * pr.yk_1 - pr.d3 * pr.yk_2 + pr.n1 * uk + pr.n2 * pr.uk_1 + pr.n3 * pr.uk_2) / pr.d1;
  pr.yk_2 = pr.yk_1;
  pr.yk_1 = yk;
  pr.uk_2 = pr.uk_1;
  pr.uk_1 = uk;
  return yk;
}

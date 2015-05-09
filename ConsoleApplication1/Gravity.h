#include "include/gfx/vec2.h"
#include "Particle.h"
#include "Force.h"

class Gravity : public Force
{

public:

	Gravity(Particle *p1, const Vec2f & grav);

	void draw();
	void apply();

	Particle * const m_p1;
	Vec2f const g;
};

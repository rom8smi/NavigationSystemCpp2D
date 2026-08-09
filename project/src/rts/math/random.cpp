#include "random.hpp"

// Linear congruential generator based on https://stackoverflow.com/questions/10198758/how-to-get-current-seed-from-c-rand

namespace NavigationSystemCode
{
	Random::Random()
	{
		seed = 0;
		random_max = 32767.0f;
	}

	Random::Random(unsigned long p_seed)
	{
		seed = p_seed;
		random_max = 32767.0f;
	}

	int Random::next_int()
	{
		seed = seed * 1103515245 + 12345;
		return (unsigned int)(seed / 65536) % 32768;
	}

	// min and max - inclusive
	int Random::next_int(int min, int max)
	{
		return (int)next_float(min, max + 1.0f);
	}

	float Random::next_float()
	{
		return float(next_int()) / random_max;
	}

	float Random::next_float(float min, float max)
	{
		return (max - min) * next_float() + min;
	}

	unsigned long Random::get_seed()
	{
		return seed;
	}

	void Random::set_seed(unsigned long p_seed)
	{
		seed = p_seed;
	}
}

#ifndef RANDOM_HPP
#define RANDOM_HPP

namespace NavigationSystemCode
{
	// Linear congruential generator based on https://stackoverflow.com/questions/10198758/how-to-get-current-seed-from-c-rand
	struct Random
	{
		unsigned long seed;
		float random_max;

		Random();
		Random(unsigned long p_seed);
		int next_int();
        int next_int(int min, int max);
        float next_float();
        float next_float(float min, float max);
		unsigned long get_seed();
		void set_seed(unsigned long p_seed);
	};
}

#endif

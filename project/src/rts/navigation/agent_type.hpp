#ifndef AGENT_TYPE_HPP
#define AGENT_TYPE_HPP

namespace NavigationSystemCode
{
    struct AgentType
    {
        float radius;
        bool isStatic;
        float speed;
        bool softLocalAvoidance;
    };
}

#endif

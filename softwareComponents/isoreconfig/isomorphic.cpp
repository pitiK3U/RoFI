#include <configuration/rofibot.hpp>

using Positions = std::set< Matrix >;

Positions decomposeRofibot( const Rofibot& rb )
{
    rb.prepare();
    assert( rb.isValid() );

    Positions result;

    for ( ModuleInfo& mod : rb.modules() )
    {
        for ( const Component& comp : mod.components() )
        {
            result.insert( comp.getPosition() );
        }
    }

    return result;
}

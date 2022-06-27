#include <isoreconfig/isomorphic.hpp>
#include <cassert>

Positions decomposeRofibot( Rofibot rb )
{
    rb.prepare();
    auto [ ok, err ] = rb.isValid();
    assert( ok );

    Positions result;

    for ( const /*Rofibot::ModuleInfo*/ auto& modInf : rb.modules() )
    {
        for ( const Component& comp : modInf.module->components() )
        {
            result.push_back( comp.getPosition() );
        }
    }

    return result;
}

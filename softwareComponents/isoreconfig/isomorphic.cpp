using ModuleMap = std::unordered_map<ID, Module>;
using Points = std::vector< std::tuple< double, double, double > >

Points modToPoints( const Module &mod )
{
    Points result;
    mod.matrices
}

Points confToPoints( const Configuration &conf )
{
    ModuleMap modules = conf.getModules();
    Points result;

    for ( Module &m : modules )
    {
        
    }
    
    return result;
}


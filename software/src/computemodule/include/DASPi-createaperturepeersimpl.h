// DASPi-createaperturepeersimpl.h



template<class SphereSpaceType, std::size_t... FaceIndices>
auto CreateAperturePeersImpl(
    const ProgramOptions& options,
    const NetworkAddressPlan& addressPlan,
    std::integer_sequence<std::size_t, FaceIndices...>
) -> std::vector<std::unique_ptr<DASPi::AperturePeerBase>>
{
    std::vector<std::unique_ptr<DASPi::AperturePeerBase>> peers;

    peers.reserve(sizeof...(FaceIndices));

    std::size_t moduleIndex = 0;

    (
        peers.push_back(
            std::make_unique<DASPi::AperturePeer<static_cast<unsigned int>(FaceIndices)>>(
                options,
                addressPlan,
                moduleIndex++
            )
        ),
        ...
    );

    return peers;
}

template<class SphereSpaceType>
auto CreateAperturePeers(
    const ProgramOptions& options,
    const NetworkAddressPlan& addressPlan
) -> std::vector<std::unique_ptr<DASPi::AperturePeerBase>>
{
    static_assert(DASPi::IcosahedronSphereSpace_t<SphereSpaceType>);

    return CreateAperturePeersImpl<SphereSpaceType>(
        options,
        addressPlan,
        typename SphereSpaceType::ModuleFaceIndices_t{}
    );
}

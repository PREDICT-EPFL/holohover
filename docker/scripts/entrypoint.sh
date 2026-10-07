#!/bin/bash
set -e

# Fast DDS super-client profile, so that ros2 CLI tools see every node registered on the discovery server
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ] && [ -n "${ROS_DISCOVERY_SERVER:-}" ]; then
    cat > "$FASTRTPS_DEFAULT_PROFILES_FILE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="super_client_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                <metatrafficUnicastLocatorList>
                                    <locator>
                                        <udpv4>
                                            <address>${ROS_DISCOVERY_SERVER%:*}</address>
                                            <port>${ROS_DISCOVERY_SERVER##*:}</port>
                                        </udpv4>
                                    </locator>
                                </metatrafficUnicastLocatorList>
                            </RemoteServer>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
EOF
fi

source /root/setup.bash
exec "$@"

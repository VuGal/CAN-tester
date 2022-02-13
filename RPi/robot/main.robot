*** Settings ***

Documentation    CAN Project
Library    OperatingSystem
Library    Dialogs
Library    Collections
Library    python/Keywords.py


*** Variables ***

${TIMEOUT}    30
${CONFIG_PATH}    /home/pi/CAN_Tester/config.json


*** Test Cases ***

Initialize CAN
    [Tags]    required
    ${CanBus}    ${ExtendedID}    Initialize CAN
    Set Suite Variable    ${CanBus}
    Set Suite Variable    ${ExtendedID}

Test Sniffing Config Mode Test
    [Tags]    sniffing_config_mode
    Test Sniffing Config Mode Keyword

Test Sniffing Full Scan Mode
    [Tags]    sniffing_full_scan_mode
    Run Keyword And Ignore Error    Test Sniffing Full Scan Mode Keyword

Test Node Config Mode Keyword
    [Tags]    node_config_mode
    Test Node Config Mode Keyword

Test Node Full Scan Mode Keyword
    [Tags]    node_full_scan_mode
    Run Keyword And Ignore Error    Test Node Full Scan Mode Keyword

Deinitialize CAN
    [Tags]    required
    Deinitialize CAN


*** Keywords ***

Test Sniffing Config Mode Keyword
    ${messages}    Collect Messages    ${CanBus}    ${30}
    ${data}    Read Json File    ${CONFIG_PATH}
    FOR    ${Unit}    IN    @{data}[SniffMode]
        ${Set}    ${UnitObj}    Test Sniffing Config Mode    ${CanBus}    ${Unit}    ${data}    ${messages}
        FOR     ${ErrorCode}    IN    @{Set}
            Run Keyword And Continue On Failure    Test Sniffing Config Mode Label Code Error    ${ErrorCode}    ${UnitObj}
        END
    END

Test Sniffing Full Scan Mode Keyword
    ${messages}    Collect Messages    ${CanBus}    ${30}
    FOR    ${Packet}    IN    @{messages}
        Log    ${Packet}
    END

Test Node Config Mode Keyword
    ${data}    Read Json File    ${CONFIG_PATH}
    FOR    ${Unit}    IN    @{data}[NodeMode]
        Run Keyword And Continue On Failure    Test Node Config Mode Send Message    ${CanBus}    ${Unit}    ${data}    ${ExtendedID}
    END


Test Node Full Scan Mode Keyword
    ${messages}    Test Node Full Scan Mode Send Messages    ${CanBus}    ${ExtendedID}
    ${messages_keys}    Get Dictionary Keys    ${messages}    sort_keys=False
    FOR    ${ID}    IN    @{messages_keys}
        Run Keyword And Continue On Failure    Test Node Full Scan Mode Label Code Error    ${ID}    ${messages}
    END

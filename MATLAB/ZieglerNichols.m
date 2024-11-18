function [KP, KI, KD, Ti, Td] = ZieglerNichols(KU, TU, TYPE)
    % This function calculates PID parameters using the Ziegler-Nichols method.
    % Inputs:
    %   KU - Ultimate gain (KU)
    %   TU - Oscillation period (TU)
    %   TYPE - Type of controller ('ClassicPID', 'P', 'PI', 'PD', 'PessenIntegrationRule', 'SomeOvershoot', 'NoOvershoot')
    
    % Ensure that TU is a positive value
    if TU <= 0
        error('TU must be a positive value');
    end

    % Default calculations for each controller type
    switch TYPE
        case 'ClassicPID'
            KP = 0.6 * KU;
            Ti = TU / 2;
            Td = TU / 8;
        case 'P'
            KP = 0.5 * KU;
            Ti = NaN;
            Td = NaN;
        case 'PI'
            KP = 0.45 * KU;
            Ti = TU / 1.2;
            Td = NaN;
        case 'PD'
            KP = 0.8 * KU;
            Ti = NaN;
            Td = TU / 8;
        case 'PessenIntegrationRule'
            KP = 0.7 * KU;
            Ti = 2 * TU / 5;
            Td = 3 * TU / 20;
        case 'SomeOvershoot'
            KP = KU / 3;
            Ti = TU / 2;
            Td = TU / 3;
        case 'NoOvershoot'
            KP = 0.2 * KU;
            Ti = TU / 2;
            Td = TU / 3;
        otherwise
            error('Unsupported TYPE');
    end
    
    % Calculate KI and KD based on KP, Ti, and Td
    KI = KP / Ti;
    KD = Td * KP;

    % If KI or KD are NaN, set them to 0
    if isnan(KI)
        KI = 0;
    end
    if isnan(KD)
        KD = 0;
    end
end

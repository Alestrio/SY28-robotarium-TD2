%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% This script defines functions for the propagation models of the communications between the agents.  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% We assume that the communication between the agents is based on NB-IoT technology, using especially %%
%% the band B4 (1710MHz + 45MHz BW). The propagation model is based on the Walfish-Ikegami model,      %%
%% allowing us to calculate different attenuations depending on the distance between the agents, and   %%
%% the presence of a Line of Sight (LoS) between them. We assume agents operate in a urban environment.%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Code is based on the author's implementation of the Walfish-Ikegami, linked to the report as        %%
%% reference.                                                                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Author: Alexis LEBEL, Date: 2024-12-03                                                              %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Explanation of acronyms:
% LOS: Line of Sight
% NLOS: Non-Line of Sight
% RTS: roof-to-street diffraction and scatter loss
% MSD: multiscreen diffraction loss
% L_ORI: Losses due to the orientation of the antenna regarding the street
% L_BSH: Losses due to the building shadowing
% KA: Losses due to the antenna height
% KF: Losses due to the frequency
% KD: Losses due to the distance between the transmitter and the receiver

classdef PropagationModel
    methods (Static)
        %% Friis Model (Prerequisite for Walfish-Ikegami NLOS)
        function friis_attenuation = Friis(distance, frequency)
            friis_attenuation = 32.4 + 20*log10(distance) + 20*log10(frequency);
        end

        %% Walfish-Ikegami
        function los_attenuation = WalfishIkegami_LOS(distance, frequency)
            if frequency <= 2000 && frequency >= 800 % && distance >= 0.02
                los_attenuation = 42.64 + 26*log10(distance) + 20*log10(frequency);
            end
        end

        function nlos_ori = wi_nlos_l_ori(angle)
            if angle >= 0 && angle < 35
                nlos_ori = -10 + 0.354 * angle;
            elseif angle >= 35 && angle < 55
                nlos_ori = 2.5 + 0.075 * (angle - 35);
            else
                nlos_ori = 4 - 0.114 * (angle - 55);
            end
        end

        function nlos_rts = wi_nlos_l_rts(street_width, frequency, building_height, rx_height, angle)
            nlos_rts = (-8.2 - 10*log10(street_width) + 10*log10(frequency) + 20*log10(building_height - rx_height) + PropagationModel.wi_nlos_l_ori(angle));
        end

        function l_bsh = wi_nlos_l_bsh(tx_height, building_height)
            if tx_height > building_height
                l_bsh = -18 * log10(1 + (tx_height - building_height));
            else
                l_bsh = 0;
            end
        end

        function ka = wi_nlos_ka(building_height, tx_height, distance)
            if tx_height > building_height
                ka = 54;
            elseif tx_height <= building_height && distance >= 0.5
                ka = 54 - 0.8 * (tx_height - building_height);
            else
                ka = 54 - 0.8 * (tx_height - building_height) * distance * 2;
            end
        end

        function kf = wi_nlos_kf(frequency)
            kf = -4 + 1.5 * ((frequency / 925) - 1);
        end

        function kd = wi_nlos_kd(building_height, tx_height)
            if tx_height > building_height
                kd = 18;
            else
                kd = 18 - 15 * (tx_height - building_height) / building_height;
            end
        end

        function msd = wi_nlos_l_msd(frequency, building_height, tx_height, distance, in_between_building_distance)
            l_bsh = PropagationModel.wi_nlos_l_bsh(tx_height, building_height);
            ka = PropagationModel.wi_nlos_ka(building_height, tx_height, distance);
            kd = PropagationModel.wi_nlos_kd(building_height, tx_height);
            kf = PropagationModel.wi_nlos_kf(frequency);

            msd = l_bsh + ka + kd * log10(distance) + kf * log10(frequency) - 9 * log10(in_between_building_distance);
        end

        function nlos_attenuation = WalfishIkegami_NLOS(street_width, frequency, building_height, rx_height, angle, tx_height, distance, in_between_building_distance)
            l_rts = PropagationModel.wi_nlos_l_rts(street_width, frequency, building_height, rx_height, angle);
            l_msd = PropagationModel.wi_nlos_l_msd(frequency, building_height, tx_height, distance, in_between_building_distance);
            friis = PropagationModel.Friis(distance, frequency);

            if l_rts + l_msd > 0
                nlos_attenuation = friis + l_rts + l_msd;
            else
                nlos_attenuation = friis;
            end
        end
    end
end

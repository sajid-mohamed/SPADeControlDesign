function [timing_pattern] = expressionToTimingPattern(pattern,maxScenarioValue)
% EXPRESSIONTOTIMINGPATTERN - Automatically change workload pattern{} expressions to
%                             timing_pattern{} for MATLAB simulation
%   Arguments:
%       pattern: Cell array of workload sequences to simulate. pattern{i}
%                contains the workload sequence of i-th pattern.
%                Some examples for pattern{i}: 
%                                  pattern{1}=1;
%                                  pattern{2}='s_2';
%                                  pattern{3}='s_3';
%                                  pattern{4}='s_2^{6} s_3^{6} s_1^{12}';
%                                  pattern{5}='s_1 s_2 s_3';
%                                  pattern{6}='s_2 s_1^{6} s_2';
%                                  pattern{7}=[1 2 3];
%       maxScenarioValue: The maximum number of the allowed sequence. E.g. if
%                maxScenarioValue=2, pattern{3}='s_3' will throw an error for 
%                unknown scenario. Only scenario value < maxScenarioValue is acceptable 
%   Returns:
%       timing_pattern: Pattern in flattened double array format that
%                       enables seamless Matlab simulation. 
%                       E.g. pattern{5}='s_1 s_2 s_3'; (transforms to)-->
%                                       timing_pattern{5}=[1 2 3]
%                            pattern{6}='s_2 s_1^{6} s_2'; -->
%                                       timing_pattern{6}=[2 1 1 1 1 1 1 2]
%   Usage:
%       EXPRESSIONTOTIMINGPATTERN(pattern,maxScenarioValue)
%
% Author: Sajid Mohamed

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Default argument values
if nargin < 2
    error('Not enough input arguments for simulation. For details - type >>help expressionToTimingPattern');
end
%% BEGIN: AUTOMATICALLY CHANGE pattern{} to timing_pattern{} for simulation
% For example:
% pattern{1}= 's_1' --> timing_pattern{1}=[1]; 
% pattern{2}= 's_1^10 s_3' --> timing_pattern{2}=[1 1 1 1 1 1 1 1 1 1 3];
% pattern{2}= 's_2^10 s_3' --> timing_pattern{3}=[2 2 2 2 2 2 2 2 2 2 3];
% tauSysScenarios: needed for checking the size alone. may be only
% length(tauSysScenarios) needed for the function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
timing_pattern=cell(1,length(pattern)); %pre-allocate the timing_pattern
for i=1:length(pattern)
    timing_pattern{i}=[];
    %Find the index of the scenarios in the string pattern
    if isa(pattern{i},'char')
        index=strfind(pattern{i},'s_');
        for j=1:length(index)
            %Find the system scenario: sValue; sValue corresponds to SystemScenario tauSystemScenarios(sValue)
            sValue=sscanf(pattern{i}(index(j) + length('s_'):end), '%g', 1);
            if sValue>maxScenarioValue
                error('The number of input scenarios you added in tauWorkloadScenarios = %d.\nThe input pattern sequence for pattern{%d} has an unknown scenario: %d',maxScenarioValue,i,sValue);
            end
            check=sprintf('s_%d^{',sValue);
            %Find the index of the scenario repetitions in the string pattern
            repeat=strfind(pattern{i},check);
            if ismember(index(j),repeat)
                %Find the repetition rate: rValue
                rValue=sscanf(pattern{i}(index(j) + length(check):end), '%g', 1); %length('s_')+length(num2str(sValue))
                for l=1:rValue
                    %Generate timing_pattern
                    timing_pattern{i}=[timing_pattern{i} sValue];
                end
            else
                timing_pattern{i}=[timing_pattern{i} sValue];
            end               
        end
    elseif isa(pattern{i},'double') % pattern represented as an array
        timing_pattern{i}=pattern{i};
        if max(pattern{i})>maxScenarioValue
            error('The input pattern sequence for pattern{%d} has unknown scenario(s)'); 
        end
    else
        error('Undefined pattern{} format for SIMULATION');
    end
end
%END: AUTOMATICALLY CHANGE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
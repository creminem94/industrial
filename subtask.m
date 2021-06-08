classdef subtask
    
    properties
        startPos
        endPos
        taskTime
        type %move/assemble
        Dk %kuka potential incapability coeff
        Da %abb
        Pk %kuka potential insufficeincy coeff for precios
        Pa %abb
        description
    end
    
    methods
        function obj = subtask(startPos,endPos,taskTime,type,Dk,Da,Pk,Pa,description)
            obj.startPos = startPos;
            obj.endPos = endPos;
            obj.taskTime = taskTime;
            obj.type = type;
            obj.Dk = Dk;
            obj.Da = Da;
            obj.Pk = Pk;
            obj.Pa = Pa;
            if nargin == 9
                obj.description = description;
            else
                obj.description = "";
            end
        end
%         
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 Summary of this method goes here
%             %   Detailed explanation goes here
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end


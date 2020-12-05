function in = walkerResetFn(in)
    in.setVariable('BRUpper',(rand(1)-0.5)/0.5);
    in.setVariable('BLUpper',(rand(1)-0.5)/0.5);
    in.setVariable('BRLower',(rand(1)-0.5)/0.5);
    in.setVariable('BLLower',(rand(1)-0.5)/0.5);
    in.setVariable('FRUpper',(rand(1)-0.5)/0.5);
    in.setVariable('FLUpper',(rand(1)-0.5)/0.5);
    in.setVariable('FRLower',(rand(1)-0.5)/0.5);
    in.setVariable('FLLower',(rand(1)-0.5)/0.5);
end
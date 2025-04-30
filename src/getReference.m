function XRef_MPC = getReference(XRef, step, N, ns, refLen)
%GETREFERENCE Summary of this function goes here
%   Detailed explanation goes here

    start_idx = mod((ns*(step-1)) + 1, refLen);

    end_idx = mod((ns*(step-1)) + ((N+1) *ns), refLen);

    if end_idx == 0
        end_idx = refLen;
    end

    if start_idx > end_idx % in this case, it has wrapped around

        XRef_MPC = XRef(start_idx: refLen);
        XRef_MPC = [XRef_MPC; XRef(1:end_idx)];
    else
        XRef_MPC = XRef(start_idx: end_idx);
    end
end


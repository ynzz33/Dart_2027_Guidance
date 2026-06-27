function R = dcm_ned2body(phi, theta, psi)
% =========================================================================
% DCM_NED2BODY  方向余弦矩阵（DCM）：把 NED 大地系的矢量，旋转到机体系
% -------------------------------------------------------------------------
% 【这个矩阵干什么】同一个矢量（比如重力、风速），在“大地系”和“机体系”里
%   分量不一样。这个 3×3 矩阵 R 就是两者之间的“翻译器”：
%       v_机体 = R · v_NED        （NED → 机体）
%       v_NED  = R' · v_机体       （机体 → NED，正交矩阵的逆=转置，所以用 R'）
%
% 【欧拉角顺序 ZYX（航空标准 yaw→pitch→roll）】从大地系到机体系依次转三次：
%       先绕 Z 转 ψ(偏航) → 再绕 Y 转 θ(俯仰) → 最后绕 X 转 φ(滚转)
%   下面的矩阵就是这三次旋转矩阵乘起来的结果（已展开成显式三角函数，省去运行时矩阵相乘）。
%
% 【入参】phi=滚转 φ, theta=俯仰 θ, psi=偏航 ψ（单位：rad）
% 【返回】R(3×3) NED→机体 的方向余弦矩阵
% =========================================================================
    cphi=cos(phi); sphi=sin(phi);       % 滚转角的 cos/sin
    cth =cos(theta); sth=sin(theta);    % 俯仰角的 cos/sin
    cpsi=cos(psi); spsi=sin(psi);       % 偏航角的 cos/sin
    R = [ cth*cpsi,                 cth*spsi,                -sth;
          sphi*sth*cpsi-cphi*spsi,  sphi*sth*spsi+cphi*cpsi,  sphi*cth;
          cphi*sth*cpsi+sphi*spsi,  cphi*sth*spsi-sphi*cpsi,  cphi*cth ];
end

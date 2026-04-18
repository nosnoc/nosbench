function save_qpec(qpec, name)
    qpec_struct.Q = sparse(qpec.Q);
    qpec_struct.A = sparse(qpec.A);
    qpec_struct.G = sparse(qpec.G);
    qpec_struct.H = sparse(qpec.H);

    qpec_struct.q = qpec.q;
    qpec_struct.lbw = qpec.lbw;
    qpec_struct.ubw = qpec.ubw;
    qpec_struct.b = qpec.b;
    qpec_struct.lba = qpec.lba;
    qpec_struct.uba = qpec.uba;
    qpec_struct.g = qpec.g;
    qpec_struct.h = qpec.h;

    qpec_struct.w0 = qpec.w0;
    qpec_struct.y0 = qpec.y0;

    save([name,'.mat'], "qpec_struct", "name");
end

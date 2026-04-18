function [qpec,name] = load_qpec(filename)
    load(filename, "qpec_struct", "name");

    qpec = nosnoc.qpec.Qpec(qpec_struct.Q, qpec_struct.A, qpec_struct.G, qpec_struct.H);

    qpec.q = qpec_struct.q;
    qpec.lbw = qpec_struct.lbw;
    qpec.ubw = qpec_struct.ubw;
    qpec.b = qpec_struct.b;
    qpec.lba = qpec_struct.lba;
    qpec.uba = qpec_struct.uba;
    qpec.g = qpec_struct.g;
    qpec.h = qpec_struct.h;

    qpec.w0 = qpec_struct.w0;
    qpec.y0 = qpec_struct.y0;
end

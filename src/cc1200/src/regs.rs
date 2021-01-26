#![rustfmt::skip]

use drone_core::bitfield::Bitfield;

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    gpio3_cfg(rw, 0, 6),
    gpio3_inv(rw, 6),
    gpio3_atran(rw, 7),
)]
pub struct Iocfg3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    gpio2_cfg(rw, 0, 6),
    gpio2_inv(rw, 6),
    gpio2_atran(rw, 7),
)]
pub struct Iocfg2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    gpio1_cfg(rw, 0, 6),
    gpio1_inv(rw, 6),
    gpio1_atran(rw, 7),
)]
pub struct Iocfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    gpio0_cfg(rw, 0, 6),
    gpio0_inv(rw, 6),
    gpio0_atran(rw, 7),
)]
pub struct Iocfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    sync31_24(rw, 0, 8),
)]
pub struct Sync3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    sync23_16(rw, 0, 8),
)]
pub struct Sync2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    sync15_8(rw, 0, 8),
)]
pub struct Sync1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    sync7_0(rw, 0, 8),
)]
pub struct Sync0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    sync_thr(rw, 0, 5),
    sync_mode(rw, 5, 3),
)]
pub struct SyncCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    strict_sync_check(rw, 0, 2),
    ext_sync_detect(rw, 2),
    pqt_gating_en(rw, 3),
    rx_config_limitation(rw, 4),
    auto_clear(rw, 5),
    not_used(r, 6, 2),
)]
pub struct SyncCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dev_m(rw, 0, 8),
)]
pub struct DeviationM(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dev_e(rw, 0, 3),
    mod_format(rw, 3, 3),
    modem_mode(rw, 6, 2),
)]
pub struct ModcfgDevE(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dcfilt_bw(rw, 0, 3),
    dcfilt_bw_settle(rw, 3, 3),
    dcfilt_freeze_coeff(rw, 6),
    not_used(r, 7),
)]
pub struct DcfiltCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    preamble_word(rw, 0, 2),
    num_preamble(rw, 2, 4),
    not_used(r, 6, 2),
)]
pub struct PreambleCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    pqt(rw, 0, 4),
    pqt_valid_timeout(rw, 4, 3),
    pqt_en(rw, 7),
)]
pub struct PreambleCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    iqic_imgch_level_thr(rw, 0, 2),
    iqic_blen(rw, 2, 2),
    iqic_blen_settle(rw, 4, 2),
    iqic_update_coeff_en(rw, 6),
    iqic_en(rw, 7),
)]
pub struct Iqic(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    bb_cic_decfact(rw, 0, 6),
    adc_cic_decfact(rw, 6, 2),
)]
pub struct ChanBw(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    cfm_data_en(rw, 0),
    upsampler_p(rw, 1, 3),
    symbol_map_cfg(rw, 4, 2),
    ask_shape(rw, 6, 2),
)]
pub struct Mdmcfg2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    single_adc_en(rw, 0),
    dvga_gain(rw, 1, 2),
    collision_detect_en(rw, 3),
    invert_data_en(rw, 4),
    manchester_en(rw, 5),
    fifo_en(rw, 6),
    carrier_sense_gate(rw, 7),
)]
pub struct Mdmcfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    viterbi_en(rw, 2),
    data_filter_en(rw, 3),
    transparent_intfact(rw, 4, 2),
    transparent_mode_en(rw, 6),
    reserved7(rw, 7),
)]
pub struct Mdmcfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    srate_m_19_16(rw, 0, 4),
    srate_e(rw, 4, 4),
)]
pub struct SymbolRate2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    srate_m_15_8(rw, 0, 8),
)]
pub struct SymbolRate1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    srate_m_7_0(rw, 0, 8),
)]
pub struct SymbolRate0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_reference(rw, 0, 8),
)]
pub struct AgcRef(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_cs_th(rw, 0, 8),
)]
pub struct AgcCsThr(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    gain_adjustment(rw, 0, 8),
)]
pub struct AgcGainAdjust(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_min_gain(rw, 0, 5),
    agc_sync_behaviour(rw, 5, 3),
)]
pub struct AgcCfg3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_max_gain(rw, 0, 5),
    fe_performance_mode(rw, 6),
    start_previous_gain_en(rw, 7),
)]
pub struct AgcCfg2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_settle_wait(rw, 0, 3),
    agc_win_size(rw, 3, 3),
    rssi_step_thr(rw, 6),
    not_used(r, 7),
)]
pub struct AgcCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_ask_decay(rw, 0, 2),
    rssi_valid_cnt(rw, 2, 2),
    agc_slewrate_limit(rw, 4, 2),
    agc_hyst_level(rw, 6, 2),
)]
pub struct AgcCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    fifo_thr(rw, 0, 7),
    crc_autoflush(rw, 7),
)]
pub struct FifoCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    device_addr(rw, 0, 8),
)]
pub struct DevAddr(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    fsreg_time(rw, 0),
    lock_time(rw, 1, 2),
    fs_autocal(rw, 3, 2),
    not_used(r, 5, 3),
)]
pub struct SettlingCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    fsd_bandselect(rw, 0, 4),
    fs_lock_en(rw, 4),
    not_used(r, 5, 3),
)]
pub struct FsCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    event1(rw, 0, 3),
    wor_mode(rw, 3, 3),
    wor_res(rw, 6, 2),
)]
pub struct WorCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rc_pd(rw, 0),
    rc_mode(rw, 1, 2),
    event2_cfg(rw, 3, 2),
    div_256hz_en(rw, 5),
    rx_duty_cycle_mode(r, 6, 2),
)]
pub struct WorCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    event0_15_8(rw, 0, 8),
)]
pub struct WorEvent0Msb(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    event0_7_0(rw, 0, 8),
)]
pub struct WorEvent0Lsb(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rx_duty_cycle_time(rw, 0, 8),
)]
pub struct RxdcmTime(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    pkt_format(rw, 0, 2),
    cca_mode(rw, 2, 3),
    fg_mode_en(rw, 5),
    byte_swap_en(rw, 6),
    not_used(r, 7),
)]
pub struct PktCfg2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    append_status(rw, 0),
    crc_cfg(rw, 1, 2),
    addr_check_cfg(rw, 3, 2),
    pn9_swap_en(rw, 5),
    white_data(rw, 5),
    fec_en(rw, 6),
)]
pub struct PktCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    uart_swap_en(rw, 0),
    uart_mode_en(rw, 1),
    pkt_bit_len(rw, 2, 3),
    length_config(rw, 5, 2),
    reserved7(rw, 7),
)]
pub struct PktCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rx_time_qual(rw, 0),
    rx_time(rw, 1, 3),
    rxoff_mode(rw, 4, 2),
    not_used(r, 6, 2),
)]
pub struct RfendCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    ant_div_rx_term_cfg(rw, 0, 3),
    term_on_bad_packet_en(rw, 3),
    txoff_mode(rw, 4, 2),
    cal_end_wake_up_en(rw, 6),
    not_used(r, 7),
)]
pub struct RfendCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    pa_power_ramp(rw, 0, 6),
    pa_ramp_shape_en(rw, 6),
    not_used(r, 7),
)]
pub struct PaCfg1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    ramp_shape(rw, 0, 2),
    second_ipl(rw, 2, 3),
    first_ipl(rw, 5, 3),
)]
pub struct PaCfg0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    ask_depth(rw, 0, 6),
    agc_ask_bw(rw, 6, 2),
)]
pub struct AskCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    packet_length(rw, 0, 8),
)]
pub struct PktLen(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    cmix_cfg(rw, 2, 3),
    not_used(r, 5, 3),
)]
pub struct IfMixCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    foc_ki_factor(rw, 0, 2),
    foc_limit(rw, 2),
    foc_cfg(rw, 3, 2),
    foc_en(rw, 5),
    not_used(r, 6, 2),
)]
pub struct FreqoffCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    toc_post_sync_blocklen(rw, 0, 3),
    toc_pre_sync_blocklen(rw, 3, 3),
    toc_limit(rw, 6, 2),
)]
pub struct TocCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    aes_commands(rw, 0, 4),
    not_used(r, 4, 4),
)]
pub struct MarcSpare(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    ext_clock_freq(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct EcgCfg(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    burst_addr_incr_en(rw, 0),
    ext_40k_clock_en(rw, 1),
    pin_ctrl_en(rw, 2),
    not_used(r, 3, 5),
)]
pub struct ExtCtrl(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rcc_fine(rw, 0, 7),
    not_used(r, 7),
)]
pub struct RccalFine(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rcc_course(rw, 0, 7),
    not_used(r, 7),
)]
pub struct RccalCourse(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved4_0(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct RccalOffset(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freq_off_15_8(rw, 0, 8),
)]
pub struct Freqoff1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freq_off_7_0(rw, 0, 8),
)]
pub struct Freqoff0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freq_23_16(rw, 0, 8),
)]
pub struct Freq2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freq_15_8(rw, 0, 8),
)]
pub struct Freq1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freq_7_0(rw, 0, 8),
)]
pub struct Freq0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved3_0(rw, 0, 4),
    not_used(r, 4, 4),
)]
pub struct IfAdc2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct IfAdc1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct IfAdc0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct FsDig1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    tx_lpf_bw(rw, 0, 2),
    rx_lpf_bw(rw, 2, 2),
    reserved7_4(rw, 4, 4),
)]
pub struct FsDig0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    kvco_high_res_cfg(rw, 6),
    fs_cal3_(rw, 7),
)]
pub struct FsCal3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct FsCal2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct FsCal1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    lock_cfg(rw, 2, 2),
    not_used(r, 4, 4),
)]
pub struct FsCal0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct FsChp(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    not_used(r, 2, 6),
)]
pub struct FsDivtwo(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved2_0(rw, 0, 3),
    not_used(r, 3, 5),
)]
pub struct FsDsm1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct FsDsm0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct FsDvc1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved4_0(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct FsDvc0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    not_used(r, 0, 8),
)]
pub struct FsLbi(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved6_0(rw, 0, 7),
    not_used(r, 7),
)]
pub struct FsPfd(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved6_0(rw, 0, 7),
    not_used(r, 7),
)]
pub struct FsPre(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved4_0(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct FsRegDivCml(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct FsSpare(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved4_0(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct FsVco4(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved0(rw, 0),
    not_used(r, 1, 7),
)]
pub struct FsVco3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved6_0(rw, 0, 7),
    not_used(r, 7),
)]
pub struct FsVco2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    fsd_vcdac(rw, 2, 6),
)]
pub struct FsVco1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct FsVco0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct Gbias6(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved3_0(rw, 0, 4),
    not_used(r, 4, 4),
)]
pub struct Gbias5(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct Gbias4(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved5_0(rw, 0, 6),
    not_used(r, 6, 2),
)]
pub struct Gbias3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved6_0(rw, 0, 7),
    not_used(r, 7),
)]
pub struct Gbias2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved4_0(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct Gbias1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    not_used(r, 2, 6),
)]
pub struct Gbias0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    ifamp_bw(rw, 2, 2),
    not_used(r, 4, 4),
)]
pub struct Ifamp(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    not_used(r, 2, 6),
)]
pub struct Lna(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    not_used(r, 2, 6),
)]
pub struct Rxmix(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved3_0(rw, 0, 4),
    not_used(r, 4, 4),
)]
pub struct Xosc5(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct Xosc4(pub u8);


#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct Xosc3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    xosc_core_pd_override(rw, 0),
    reserved3_1(rw, 1, 3),
    not_used(r, 4, 4),
)]
pub struct Xosc2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    xosc_stable(r, 0),
    xosc_buf_sel(rw, 1),
    reserved2(rw, 2),
    not_used(r, 3, 4),
)]
pub struct Xosc1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved1_0(rw, 0, 2),
    not_used(r, 2, 6),
)]
pub struct Xosc0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(rw, 0, 8),
)]
pub struct AnalogSpare(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved2_0(rw, 0, 3),
    not_used(r, 3, 5),
)]
pub struct PaCfg3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    wor_status_15_8(rw, 0, 8),
)]
pub struct WorTime1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    wor_status_7_0(rw, 0, 8),
)]
pub struct WorTime0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    wor_capture_15_8(rw, 0, 8),
)]
pub struct WorCapture1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    wor_capture_7_0(rw, 0, 8),
)]
pub struct WorCapture0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved3_0(rw, 0, 4),
    not_used(r, 4, 4),
)]
pub struct Bist(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dcfilt_offset_i_15_8(rw, 0, 8),
)]
pub struct DcfiltoffsetI1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dcfilt_offset_i_7_0(rw, 0, 8),
)]
pub struct DcfiltoffsetI0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dcfilt_offset_q_15_8(rw, 0, 8),
)]
pub struct DcfiltoffsetQ1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    dcfilt_offset_q_7_0(rw, 0, 8),
)]
pub struct DcfiltoffsetQ0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    iqie_i_15_8(rw, 0, 8),
)]
pub struct IqieI1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    iqie_i_7_0(rw, 0, 8),
)]
pub struct IqieI0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    iqie_q_15_8(rw, 0, 8),
)]
pub struct IqieQ1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    iqie_q_7_0(rw, 0, 8),
)]
pub struct IqieQ0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rssi_11_4(r, 0, 8),
)]
pub struct Rssi1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rssi_valid(r, 0),
    carrier_sense_valid(r, 1),
    carrier_sense(r, 2),
    rssi_3_0(r, 3, 4),
    not_used(r, 7),
)]
pub struct Rssi0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    marc_state(r, 0, 5),
    marc_2pin_state(r, 5, 2),
    not_used(r, 7),
)]
pub struct Marcstate(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    lqi(r, 0, 7),
    pkt_crc_ok(r, 7),
)]
pub struct LqiVal(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    sync_error(r, 0, 4),
    pqt_error(r, 4, 4),
)]
pub struct PqtSyncErr(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    image_found(r, 0),
    sro_indicator(r, 1, 4),
    sync_low0_high1(r, 5),
    collision_found(r, 6),
    rssi_step_found(r, 7),
)]
pub struct DemStatus(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freqoff_est_15_8(r, 0, 8),
)]
pub struct FreqoffEst1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    freqoff_est_7_0(r, 0, 8),
)]
pub struct FreqoffEst0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    agc_front_end_gain(r, 0, 7),
    not_used(r, 7),
)]
pub struct AgcGain3(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved6_0(rw, 0, 7),
    agc_drives_fe_gain(rw, 7),
)]
pub struct AgcGain2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved4_0(rw, 0, 5),
    not_used(r, 5, 3),
)]
pub struct AgcGain1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved6_0(rw, 0, 7),
    not_used(r, 7),
)]
pub struct AgcGain0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    cfm_rx_data(r, 0, 8),
)]
pub struct CfmRxDataOut(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    cfm_tx_data(rw, 0, 8),
)]
pub struct CfmTxDataIn(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    ask_soft(r, 0, 6),
    not_used(r, 6, 2),
)]
pub struct AskSoftRxData(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rndgen_value(r, 0, 7),
    rndgen_en(rw, 7),
)]
pub struct Rndgen(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    magn_16(r, 0),
    not_used(r, 1, 7),
)]
pub struct Magn2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    magn_15_8(r, 0, 8),
)]
pub struct Magn1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    magn_7_0(r, 0, 8),
)]
pub struct Magn0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    angular_9_8(r, 0, 2),
    not_used(r, 2, 6),
)]
pub struct Ang1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    angular_7_0(r, 0, 8),
)]
pub struct Ang0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    chfilt_i_16(r, 0),
    chfilt_startup_valid(r, 1),
    not_used(r, 2, 6),
)]
pub struct ChfiltI2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    chfilt_i_15_8(r, 0, 8),
)]
pub struct ChfiltI1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    chfilt_i_7_0(r, 0, 8),
)]
pub struct ChfiltI0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    chfilt_q_16(r, 0),
    not_used(r, 1, 7),
)]
pub struct ChfiltQ2(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    chfilt_q_15_8(r, 0, 8),
)]
pub struct ChfiltQ1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    chfilt_q_7_0(r, 0, 8),
)]
pub struct ChfiltQ0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    gpio_state(r, 0, 4),
    marc_gdo_state(r, 4, 4),
)]
pub struct GpioStatus(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    lock(r, 0),
    reserved6_1(rw, 1, 6),
    not_used(r, 7),
)]
pub struct FscalCtrl(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    reserved7_0(r, 0, 8),
)]
pub struct PhaseAdjust(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    partnum(r, 0, 8),
)]
pub struct Partnumber(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    partver(r, 0, 8),
)]
pub struct Partversion(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    serial_rx_clk(r, 0),
    serial_rx(r, 1),
    cfm_tx_data_clk(r, 2),
    ioc_sync_pins_en(rw, 3),
    clk40k(r, 4),
    spi_direct_access_cfg(rw, 5),
    not_used(r, 6, 2),
)]
pub struct SerialStatus(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    pqt_valid(r, 0),
    pqt_reached(r, 1),
    rxfifo_underflow(r, 2),
    rxfifo_overflow(r, 3),
    rxfifo_empty(r, 4),
    rxfifo_thr(r, 5),
    rxfifo_full(r, 6),
    sync_found(r, 7),
)]
pub struct ModemStatus1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    txfifo_underflow(r, 0),
    txfifo_overflow(r, 1),
    txfifo_thr(r, 2),
    txfifo_full(r, 3),
    sync_sent(r, 4),
    reserved5(r, 5),
    fec_rx_overflow(r, 6),
    not_used(r, 7),
)]
pub struct ModemStatus0(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    marc_status_out(r, 0, 8),
)]
pub struct MarcStatus1(pub u8);

#[derive(Clone, Copy, Bitfield)]
#[bitfield(
    rcc_cal_valid(r, 0),
    reserved1(r, 1),
    txoncca_failed(r, 2),
    reserved3(r, 3),
    not_used(r, 4, 4),
)]
pub struct MarcStatus0(pub u8);

### generic features that apply to all active-power-only (apo) approximations


"apo models ignore reactive power flows"
function variable_gen_power_imaginary(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :gen, :qg, ids(pm, nw, :gen), NaN)
end

"apo models ignore reactive power flows"
function variable_gen_power_imaginary_on_off(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :gen, :qg, ids(pm, nw, :gen), NaN)
end

"apo models ignore reactive power flows"
function variable_storage_power_imaginary(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :storage, :qs, ids(pm, nw, :storage), NaN)
end

"apo models ignore reactive power flows"
function variable_storage_power_imaginary_on_off(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :storage, :qs, ids(pm, nw, :storage), NaN)
end

"apo models ignore reactive power flows"
function variable_branch_power_imaginary(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :branch, :qf, ids(pm, nw, :branch), NaN)
    report && sol_component_fixed(pm, nw, :branch, :qt, ids(pm, nw, :branch), NaN)
end

"apo models ignore reactive power flows"
function variable_ne_branch_power_imaginary(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :ne_branch, :qf, ids(pm, nw, :ne_branch), NaN)
    report && sol_component_fixed(pm, nw, :ne_branch, :qt, ids(pm, nw, :ne_branch), NaN)
end

"apo models ignore reactive power flows"
function variable_dcline_power_imaginary(pm::AbstractActivePowerModel; nw::Int=nw_id_default, report::Bool=true, kwargs...)
    report && sol_component_fixed(pm, nw, :dcline, :qf, ids(pm, nw, :dcline), NaN)
    report && sol_component_fixed(pm, nw, :dcline, :qt, ids(pm, nw, :dcline), NaN)
end

"do nothing, apo models do not have reactive variables"
function constraint_gen_setpoint_reactive(pm::AbstractActivePowerModel, n::Int, i, qg)
end


"on/off constraint for generators"
function constraint_gen_power_on_off(pm::AbstractActivePowerModel, n::Int, i::Int, pmin, pmax, qmin, qmax)
    pg = var(pm, n, :pg, i)
    z = var(pm, n, :z_gen, i)

    JuMP.@constraint(pm.model, pg <= pmax*z)
    JuMP.@constraint(pm.model, pg >= pmin*z)
end



""
function constraint_power_balance(pm::AbstractActivePowerModel, n::Int, i::Int, bus_arcs, bus_arcs_dc, bus_arcs_sw, bus_gens, bus_storage, bus_pd, bus_qd, bus_gs, bus_bs)
    p    = get(var(pm, n),    :p, Dict()); _check_var_keys(p, bus_arcs, "active power", "branch")
    pg   = get(var(pm, n),   :pg, Dict()); _check_var_keys(pg, bus_gens, "active power", "generator")
    ps   = get(var(pm, n),   :ps, Dict()); _check_var_keys(ps, bus_storage, "active power", "storage")
    psw  = get(var(pm, n),  :psw, Dict()); _check_var_keys(psw, bus_arcs_sw, "active power", "switch")
    p_dc = get(var(pm, n), :p_dc, Dict()); _check_var_keys(p_dc, bus_arcs_dc, "active power", "dcline")


    cstr = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        + sum(p_dc[a_dc] for a_dc in bus_arcs_dc)
        + sum(psw[a_sw] for a_sw in bus_arcs_sw)
        ==
        sum(pg[g] for g in bus_gens)
        - sum(ps[s] for s in bus_storage)
        - sum(pd for pd in values(bus_pd))
        - sum(gs for gs in values(bus_gs))*1.0^2
    )

    if _IM.report_duals(pm)
        sol(pm, n, :bus, i)[:lam_kcl_r] = cstr
        sol(pm, n, :bus, i)[:lam_kcl_i] = NaN
    end
end

""
function constraint_power_balance_ls(pm::AbstractActivePowerModel, n::Int, i::Int, bus_arcs, bus_arcs_dc, bus_arcs_sw, bus_gens, bus_storage, bus_pd, bus_qd, bus_gs, bus_bs)
    p    = get(var(pm, n),    :p, Dict()); _check_var_keys(p, bus_arcs, "active power", "branch")
    pg   = get(var(pm, n),   :pg, Dict()); _check_var_keys(pg, bus_gens, "active power", "generator")
    ps   = get(var(pm, n),   :ps, Dict()); _check_var_keys(ps, bus_storage, "active power", "storage")
    psw  = get(var(pm, n),  :psw, Dict()); _check_var_keys(psw, bus_arcs_sw, "active power", "switch")
    p_dc = get(var(pm, n), :p_dc, Dict()); _check_var_keys(p_dc, bus_arcs_dc, "active power", "dcline")

    z_demand = get(var(pm, n), :z_demand, Dict()); _check_var_keys(z_demand, keys(bus_pd), "power factor", "load")
    z_shunt = get(var(pm, n), :z_shunt, Dict()); _check_var_keys(z_shunt, keys(bus_gs), "power factor", "shunt")

    cstr = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        + sum(p_dc[a_dc] for a_dc in bus_arcs_dc)
        + sum(psw[a_sw] for a_sw in bus_arcs_sw)
        ==
        sum(pg[g] for g in bus_gens)
        - sum(ps[s] for s in bus_storage)
        - sum(pd*z_demand[i] for (i,pd) in bus_pd)
        - sum(gs*z_shunt[i] for (i,gs) in bus_gs)*1.0^2
    )

    if _IM.report_duals(pm)
        sol(pm, n, :bus, i)[:lam_kcl_r] = cstr
        sol(pm, n, :bus, i)[:lam_kcl_i] = NaN
    end
end

""
function constraint_ne_power_balance(pm::AbstractDCPModel, n::Int, i, bus_arcs, bus_arcs_dc, bus_arcs_sw, bus_arcs_ne, bus_gens, bus_storage, bus_pd, bus_qd, bus_gs, bus_bs)
    p    = get(var(pm, n),    :p, Dict()); _check_var_keys(p, bus_arcs, "active power", "branch")
    pg   = get(var(pm, n),   :pg, Dict()); _check_var_keys(pg, bus_gens, "active power", "generator")
    ps   = get(var(pm, n),   :ps, Dict()); _check_var_keys(ps, bus_storage, "active power", "storage")
    psw  = get(var(pm, n),  :psw, Dict()); _check_var_keys(psw, bus_arcs_sw, "active power", "switch")
    p_dc = get(var(pm, n), :p_dc, Dict()); _check_var_keys(p_dc, bus_arcs_dc, "active power", "dcline")
    p_ne = get(var(pm, n), :p_ne, Dict()); _check_var_keys(p_ne, bus_arcs_ne, "active power", "ne_branch")

    cstr = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        + sum(p_dc[a_dc] for a_dc in bus_arcs_dc)
        + sum(psw[a_sw] for a_sw in bus_arcs_sw)
        + sum(p_ne[a] for a in bus_arcs_ne)
        ==
        sum(pg[g] for g in bus_gens)
        - sum(ps[s] for s in bus_storage)
        - sum(pd for pd in values(bus_pd))
        - sum(gs for gs in values(bus_gs))*1.0^2
    )

    if _IM.report_duals(pm)
        sol(pm, n, :bus, i)[:lam_kcl_r] = cstr
        sol(pm, n, :bus, i)[:lam_kcl_i] = NaN
    end
end

function constraint_dnep_power_balance(pm::AbstractDCPModel, n::Int, i::Int, bus_arcs, bus_arcs_sw, bus_arcs_ne, bus_gens, bus_ne_gens, bus_storage, bus_pd, bus_qd, bus_gs, bus_bs, bus_ne_storage)
    p    = get(var(pm, n),    :p, Dict()); _check_var_keys(p, bus_arcs, "active power", "branch")
    pg   = get(var(pm, n),   :pg, Dict()); _check_var_keys(pg, bus_gens, "active power", "generator")
    ps   = get(var(pm, n),   :ps, Dict()); _check_var_keys(ps, bus_storage, "active power", "storage")
    psw  = get(var(pm, n),  :psw, Dict()); _check_var_keys(psw, bus_arcs_sw, "active power", "switch")
    p_ne = get(var(pm, n), :p_ne, Dict()); _check_var_keys(p_ne, bus_arcs_ne, "active power", "ne_branch")
    pg_ne = get(var(pm, n), :pg_ne, Dict()); _check_var_keys(pg_ne, bus_ne_gens, "active power", "ne_gen")
    ps_ne = get(var(pm, n), :ps_ne, Dict()); _check_var_keys(ps_ne, bus_ne_storage, "active power", "ne_storage")

    cstr = JuMP.@constraint(pm.model,
        sum(p[a] for a in bus_arcs)
        + sum(psw[a_sw] for a_sw in bus_arcs_sw)
        + sum(p_ne[a] for a in bus_arcs_ne)
        ==
        sum(pg[g] for g in bus_gens)
        + sum(pg_ne[g] for g in bus_ne_gens)
        - sum(ps[s] for s in bus_storage)
        - sum(ps_ne[s] for s in bus_ne_storage)
        - sum(pd for pd in values(bus_pd))
        - sum(gs for gs in values(bus_gs))*1.0^2
    )

    if _IM.report_duals(pm)
        sol(pm, n, :bus, i)[:lam_kcl_r] = cstr
        sol(pm, n, :bus, i)[:lam_kcl_i] = NaN
    end
end


""
function expression_bus_power_injection(pm::AbstractActivePowerModel, n::Int, i::Int, bus_gens, bus_storage, bus_pd, bus_qd, bus_gs, bus_bs)
    pg   = get(var(pm, n),   :pg, Dict()); _check_var_keys(pg, bus_gens, "active power", "generator")
    ps   = get(var(pm, n),   :ps, Dict()); _check_var_keys(ps, bus_storage, "active power", "storage")

    pg_total = 0.0
    if length(bus_gens) > 0
        pg_total = sum(pg[g] for g in bus_gens)
    end

    ps_total = 0.0
    if length(bus_storage) > 0
        ps_total = sum(ps[s] for s in bus_storage)
    end

    pd_total = 0.0
    if length(bus_pd) > 0
        pd_total = sum(pd for pd in values(bus_pd))
    end

    gs_total = 0.0
    if length(bus_gs) > 0
        gs_total = sum(gs for gs in values(bus_gs))*1.0^2
    end

    var(pm, n, :inj_p)[i] = pg_total - ps_total - pd_total - gs_total
end


"`-rate_a <= p[f_idx] <= rate_a`"
function constraint_thermal_limit_from(pm::AbstractActivePowerModel, n::Int, f_idx, rate_a)
    p_fr = var(pm, n, :p, f_idx)
    if isa(p_fr, JuMP.VariableRef) && JuMP.has_lower_bound(p_fr)
        cstr = JuMP.LowerBoundRef(p_fr)
        JuMP.lower_bound(p_fr) < -rate_a && JuMP.set_lower_bound(p_fr, -rate_a)
        if JuMP.has_upper_bound(p_fr)
            JuMP.upper_bound(p_fr) > rate_a && JuMP.set_upper_bound(p_fr, rate_a)
        end
    else
        cstr = JuMP.@constraint(pm.model, p_fr <= rate_a)
    end

    if _IM.report_duals(pm)
        sol(pm, n, :branch, f_idx[1])[:mu_sm_fr] = cstr
    end
end

""
function constraint_thermal_limit_to(pm::AbstractActivePowerModel, n::Int, t_idx, rate_a)
    p_to = var(pm, n, :p, t_idx)
    if isa(p_to, JuMP.VariableRef) && JuMP.has_lower_bound(p_to)
        cstr = JuMP.LowerBoundRef(p_to)
        JuMP.lower_bound(p_to) < -rate_a && JuMP.set_lower_bound(p_to, -rate_a)
        if JuMP.has_upper_bound(p_to)
            JuMP.upper_bound(p_to) >  rate_a && JuMP.set_upper_bound(p_to,  rate_a)
        end
    else
        cstr = JuMP.@constraint(pm.model, p_to <= rate_a)
    end

    if _IM.report_duals(pm)
        sol(pm, n, :branch, t_idx[1])[:mu_sm_to] = cstr
    end
end


""
function constraint_current_limit_from(pm::AbstractActivePowerModel, n::Int, f_idx, c_rating_a)
    p_fr = var(pm, n, :p, f_idx)
    if isa(p_fr, JuMP.VariableRef) && JuMP.has_lower_bound(p_fr)
        JuMP.lower_bound(p_fr) < -c_rating_a && JuMP.set_lower_bound(p_fr, -c_rating_a)
        if JuMP.has_upper_bound(p_fr)
            JuMP.upper_bound(p_fr) > c_rating_a && JuMP.set_upper_bound(p_fr, c_rating_a)
        end
    else
        JuMP.@constraint(pm.model, p_fr <= c_rating_a)
    end
end

""
function constraint_current_limit_to(pm::AbstractActivePowerModel, n::Int, t_idx, c_rating_a)
    p_to = var(pm, n, :p, t_idx)
    if isa(p_to, JuMP.VariableRef) && JuMP.has_lower_bound(p_to)
        JuMP.lower_bound(p_to) < -c_rating_a && JuMP.set_lower_bound(p_to, -c_rating_a)
        if JuMP.has_upper_bound(p_to)
            JuMP.upper_bound(p_to) >  c_rating_a && JuMP.set_upper_bound(p_to,  c_rating_a)
        end
    else
        JuMP.@constraint(pm.model, p_to <= c_rating_a)
    end
end



""
function constraint_thermal_limit_from_on_off(pm::AbstractActivePowerModel, n::Int, i, f_idx, rate_a)
    p_fr = var(pm, n, :p, f_idx)
    z = var(pm, n, :z_branch, i)

    JuMP.@constraint(pm.model, p_fr <=  rate_a*z)
    JuMP.@constraint(pm.model, p_fr >= -rate_a*z)
end

""
function constraint_thermal_limit_to_on_off(pm::AbstractActivePowerModel, n::Int, i, t_idx, rate_a)
    p_to = var(pm, n, :p, t_idx)
    z = var(pm, n, :z_branch, i)

    JuMP.@constraint(pm.model, p_to <=  rate_a*z)
    JuMP.@constraint(pm.model, p_to >= -rate_a*z)
end

""
function constraint_ne_thermal_limit_from(pm::AbstractActivePowerModel, n::Int, i, f_idx, rate_a)
    p_fr = var(pm, n, :p_ne, f_idx)
    z = var(pm, n, :branch_ne, i)

    JuMP.@constraint(pm.model, p_fr <=  rate_a*z)
    JuMP.@constraint(pm.model, p_fr >= -rate_a*z)
end

""
function constraint_ne_thermal_limit_to(pm::AbstractActivePowerModel, n::Int, i, t_idx, rate_a)
    p_to = var(pm, n, :p_ne, t_idx)
    z = var(pm, n, :branch_ne, i)

    JuMP.@constraint(pm.model, p_to <=  rate_a*z)
    JuMP.@constraint(pm.model, p_to >= -rate_a*z)
end

"""
```
z[idx]*pmin[idx] <= pg_ne[idx] <= z[idx]*pmax[idx]
z[idx]*qmin[idx] <= qg_ne[idx] <= z[idx]*qmax[idx]
```
"""
function constraint_ne_gen(pm::AbstractActivePowerModel, n::Int, idx, pmin, pmax, qmin, qmax)
    pg_ne  = var(pm, n, :pg_ne, idx)
    pg_ne_loss = var(pm, n, :pg_ne_loss, idx)
    qg_ne  = var(pm, n, :qg_ne, idx)
    z_ne = var(pm, n, :gen_ne, idx)

    JuMP.@constraint(pm.model, pg_ne + pg_ne_loss == z_ne*pmax)
    JuMP.@constraint(pm.model, pg_ne >= z_ne*pmin)
    JuMP.@constraint(pm.model, pg_ne_loss >= 0)
    JuMP.@constraint(pm.model, qg_ne <= z_ne*qmax)
    JuMP.@constraint(pm.model, qg_ne >= z_ne*qmin)
end

function constraint_gen(pm::AbstractActivePowerModel, n::Int, idx, pmax)
    pg  = var(pm, n, :pg, idx)
    pg_loss = var(pm, n, :pg_loss, idx)

    JuMP.@constraint(pm.model, pg + pg_loss == pmax)
    JuMP.@constraint(pm.model, pg_loss >= 0)
end

""
function constraint_switch_thermal_limit(pm::AbstractActivePowerModel, n::Int, f_idx, rating)
    psw = var(pm, n, :psw, f_idx)

    JuMP.lower_bound(psw) < -rating && JuMP.set_lower_bound(psw, -rating)
    JuMP.upper_bound(psw) >  rating && JuMP.set_upper_bound(psw,  rating)
end

""
function constraint_storage_thermal_limit(pm::AbstractActivePowerModel, n::Int, i, rating)
    ps = var(pm, n, :ps, i)

    JuMP.lower_bound(ps) < -rating && JuMP.set_lower_bound(ps, -rating)
    JuMP.upper_bound(ps) >  rating && JuMP.set_upper_bound(ps,  rating)
end

""
function constraint_ne_storage_thermal_limit(pm::AbstractActivePowerModel, n::Int, i, rating)
    ps_ne = var(pm, n, :ps_ne, i)

    JuMP.lower_bound(ps_ne) < -rating && JuMP.set_lower_bound(ps_ne, -rating)
    JuMP.upper_bound(ps_ne) >  rating && JuMP.set_upper_bound(ps_ne,  rating)
end

""
function constraint_storage_current_limit(pm::AbstractActivePowerModel, n::Int, i, bus, rating)
    ps = var(pm, n, :ps, i)

    JuMP.lower_bound(ps) < -rating && JuMP.set_lower_bound(ps, -rating)
    JuMP.upper_bound(ps) >  rating && JuMP.set_upper_bound(ps,  rating)
end

""
function constraint_storage_losses(pm::AbstractActivePowerModel, n::Int, i, bus, r, x, p_loss, q_loss)
    ps = var(pm, n, :ps, i)
    sc = var(pm, n, :sc, i)
    sd = var(pm, n, :sd, i)

    JuMP.@constraint(pm.model, ps + (sd - sc) == p_loss + r*ps^2)
end

""
function constraint_ne_storage_losses(pm::AbstractActivePowerModel, n::Int, i, bus, r, x, p_loss, q_loss)
    ps_ne = var(pm, n, :ps_ne, i)
    sc_ne = var(pm, n, :sc_ne, i)
    sd_ne = var(pm, n, :sd_ne, i)

    JuMP.@constraint(pm.model, ps_ne + (sd_ne - sc_ne) == p_loss + r*ps_ne^2)
end

function constraint_storage_on_off(pm::AbstractActivePowerModel, n::Int, i, pmin, pmax, qmin, qmax, charge_ub, discharge_ub)
    z_storage = var(pm, n, :z_storage, i)
    ps = var(pm, n, :ps, i)

    JuMP.@constraint(pm.model, ps <= z_storage*pmax)
    JuMP.@constraint(pm.model, ps >= z_storage*pmin)
end

function constraint_ne_storage_on_off(pm::AbstractActivePowerModel, n::Int, i, pmin, pmax, qmin, qmax, charge_ub, discharge_ub)
    z_ne_storage = var(pm, n, :z_ne_storage, i)
    ps_ne = var(pm, n, :ps_ne, i)

    JuMP.@constraint(pm.model, ps <= z_ne_storage*pmax)
    JuMP.@constraint(pm.model, ps >= z_ne_storage*pmin)
end
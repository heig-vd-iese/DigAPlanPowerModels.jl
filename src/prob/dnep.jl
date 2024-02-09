#### General Assumptions of these DNEP Models ####
#
#

""
function solve_dnep(file, model_type::Type, optimizer; kwargs...)
    return solve_model(file, model_type, optimizer, build_dnep; ref_extensions=[ref_add_on_off_va_bounds!,ref_add_ne!], kwargs...)
end

"the general form of the dnep optimization model"
function build_dnep(pm::AbstractPowerModel)
    variable_bus_voltage(pm)
    variable_gen_power(pm)
    variable_branch_power_dnep(pm)

    variable_ne_branch_indicator(pm)
    variable_ne_branch_power(pm)
    variable_ne_branch_voltage(pm)

    variable_ne_gen_indicator(pm)
    variable_ne_gen_power(pm)

    objective_dnep_cost(pm)

    constraint_model_voltage(pm)
    constraint_ne_model_voltage(pm)

    for i in ids(pm, :ref_buses)
        constraint_theta_ref(pm, i)
    end

    for i in ids(pm, :bus)
        constraint_dnep_power_balance(pm, i)
    end

    for i in ids(pm, :branch)
        constraint_ohms_yt_from(pm, i)
        constraint_ohms_yt_to(pm, i)

        constraint_voltage_angle_difference(pm, i)

        constraint_thermal_limit_from_dnep(pm, i)
        constraint_thermal_limit_to_dnep(pm, i)
    end

    for i in ids(pm, :ne_branch)
        constraint_ne_ohms_yt_from(pm, i)
        constraint_ne_ohms_yt_to(pm, i)

        constraint_ne_voltage_angle_difference(pm, i)

        constraint_ne_thermal_limit_from(pm, i)
        constraint_ne_thermal_limit_to(pm, i)
    end

    for i in ids(pm, :ne_gen)
        constraint_ne_gen(pm, i)
    end

    for i in ids(pm, :gen)
        constraint_gen(pm, i)
    end

end


"Cost of building branches and new generators"
function objective_dnep_cost(pm::AbstractPowerModel)
    if haskey(ref(pm, nw_id_default), :power_flex_price)
        power_flex_price = ref(pm, nw_id_default, :power_flex_price)
    else
        power_flex_price = 1
    end
    return JuMP.@objective(pm.model, Min,
        sum(
            sum(branch["construction_cost"]*var(pm, n, :branch_ne, i) for (i,branch) in nw_ref[:ne_branch]) + 
            sum(gen["construction_cost"]*var(pm, n, :gen_ne, i) for (i,gen) in nw_ref[:ne_gen]) + 
            sum(power_flex_price*var(pm, n, :pg_loss, i) for (i,gen) in nw_ref[:gen]) + 
            sum(power_flex_price*var(pm, n, :pg_ne_loss, i) for (i,gen) in nw_ref[:ne_gen])
        for (n, nw_ref) in nws(pm))
    )
end


function solve_dnep_mn_strg(file, model_type::Type, optimizer; kwargs...)
    return solve_model(file, model_type, optimizer, build_dnep_mn_strg; ref_extensions=[ref_add_on_off_va_bounds!,ref_add_ne!], multinetwork=true, kwargs...)
end

"the general form of the dnep optimization model"
function build_dnep_mn_strg(pm::AbstractPowerModel)
    for (n, network) in nws(pm)
        variable_bus_voltage(pm, nw=n)
        variable_gen_power(pm, nw=n)
        variable_branch_power_dnep(pm, nw=n)
        variable_storage_power_mi(pm, nw=n)

        variable_ne_branch_indicator(pm, nw=n)
        variable_ne_branch_power(pm, nw=n)
        variable_ne_branch_voltage(pm, nw=n)

        variable_ne_gen_indicator(pm, nw=n)
        variable_ne_gen_power(pm, nw=n)

        variable_ne_storage_indicator(pm, nw=n)
        variable_ne_storage_power_mi_on_off(pm, nw=n)

        constraint_model_voltage(pm, nw=n)
        constraint_ne_model_voltage(pm, nw=n)

        for i in ids(pm, :ref_buses, nw=n)
            constraint_theta_ref(pm, i, nw=n)
        end

        for i in ids(pm, :bus, nw=n)
            constraint_dnep_power_balance(pm, i, nw=n)
        end

        for i in ids(pm, :storage, nw=n)
            constraint_storage_complementarity_mi(pm, i, nw=n)
            constraint_storage_losses(pm, i, nw=n)
            constraint_storage_thermal_limit(pm, i, nw=n)
        end

        for i in ids(pm, :branch, nw=n)
            constraint_ohms_yt_from(pm, i, nw=n)
            constraint_ohms_yt_to(pm, i, nw=n)

            constraint_voltage_angle_difference(pm, i, nw=n)

            constraint_thermal_limit_from_dnep(pm, i, nw=n)
            constraint_thermal_limit_to_dnep(pm, i, nw=n)
        end

        for i in ids(pm, :gen, nw=n)
            constraint_gen(pm, i, nw=n)
        end

        for i in ids(pm, :ne_branch, nw=n)
            constraint_ne_ohms_yt_from(pm, i,nw=n)
            constraint_ne_ohms_yt_to(pm, i, nw=n)

            constraint_ne_voltage_angle_difference(pm, i, nw=n)

            constraint_ne_thermal_limit_from(pm, i, nw=n)
            constraint_ne_thermal_limit_to(pm, i, nw=n)
        end

        for i in ids(pm, :ne_gen, nw=n)
            constraint_ne_gen(pm, i, nw=n)
        end

        for i in ids(pm, :ne_storage, nw=n)
            constraint_ne_storage_on_off(pm, i, nw=n)
            constraint_ne_storage_complementarity_mi(pm, i, nw=n)
            constraint_ne_storage_losses(pm, i, nw=n)
            constraint_ne_storage_thermal_limit(pm, i, nw=n)
        end

    end

    network_ids = sort(collect(nw_ids(pm)))
    network_id_first = network_ids[1]
    
    if haskey(ref(pm, network_id_first), :tid)
        tid = ref(pm, network_id_first, :tid)
    else
        tid = 24
    end

    number_scenarios = cld(length(network_ids), tid)

    for i in 1:number_scenarios
        scen_fi = (i-1)*tid
        n_1 = network_ids[scen_fi+1]
        for i in ids(pm, :storage, nw=n_1)
            constraint_storage_state(pm, i, nw=n_1)
        end
        for i in ids(pm, :ne_storage, nw=n_1)
            constraint_ne_storage_state(pm, i, nw=n_1)
        end
        for n_2 in network_ids[scen_fi*tid+2:min(scen_fi+tid, length(network_ids))]
            for i in ids(pm, :storage, nw=n_2)
                constraint_storage_state(pm, i, n_1, n_2)
            end
            for i in ids(pm, :ne_storage, nw=n_2)
                constraint_ne_storage_state(pm, i, n_1, n_2)
            end
            n_1 = n_2
        end
    end

    for n_2 in eachindex(network_ids[2:end])
        n_1 = network_ids[1]
        for i in ids(pm, :ne_branch, nw=n_2)
            constraint_ne_branch_state(pm, i, n_1, n_2)
        end
        for i in ids(pm, :ne_gen, nw=n_2)
            constraint_ne_gen_state(pm, i, n_1, n_2)
        end
        for i in ids(pm, :ne_storage, nw=n_2)
            constraint_ne_storage_built(pm, i, n_1, n_2)
        end
        # for i in ids(pm, :branch, nw=n_2)
        #     constraint_branch_rate_add(pm, i, n_1, n_2)
        # end
        n_1 = n_2
    end

    objective_dnep_mn_strg_cost(pm)

end


"Cost of building branches and new generators"
function objective_dnep_mn_strg_cost(pm::AbstractPowerModel)
    
    network_ids = sort(collect(nw_ids(pm)))
    network_id_first = network_ids[1]

    if haskey(ref(pm, network_id_first), :power_flex_price)
        power_flex_price = ref(pm, network_id_first, :power_flex_price)
    else
        power_flex_price = 1
    end

    if haskey(ref(pm, network_id_first), :tid)
        tid = ref(pm, network_id_first, :tid)
    else
        tid = 24
    end
    
    network_ids = sort(collect(nw_ids(pm)))
    number_scenarios = cld(length(network_ids), tid)

    return JuMP.@objective(pm.model, Min,
        sum(
            sum(branch["construction_cost"]*var(pm, n, :branch_ne, i) for (i,branch) in nw_ref[:ne_branch]) +
            sum(gen["construction_cost"]*var(pm, n, :gen_ne, i) for (i,gen) in nw_ref[:ne_gen]) +
            sum(storage["construction_cost"]*var(pm, n, :z_ne_storage, i) for (i,storage) in nw_ref[:ne_storage]) +
            sum(power_flex_price*var(pm, n, :pg_loss, i) for (i,gen) in nw_ref[:gen]) + 
            sum(power_flex_price*var(pm, n, :pg_ne_loss, i) for (i,gen) in nw_ref[:ne_gen])
        for (n, nw_ref) in nws(pm)) / number_scenarios
    )
end


""
function ref_add_ne!(ref::Dict{Symbol,<:Any}, data::Dict{String,<:Any})
    apply_pm!(_ref_add_ne!, ref, data)
end


""
function _ref_add_ne!(ref::Dict{Symbol,<:Any}, data::Dict{String,<:Any})
    if !haskey(ref, :ne_branch) & !haskey(ref, :ne_gen) 
        error(_LOGGER, "required ne_branch or ne_gen data not found")
    end

    ref[:ne_branch] = Dict(x for x in ref[:ne_branch] if (x.second["br_status"] == 1 && x.second["f_bus"] in keys(ref[:bus]) && x.second["t_bus"] in keys(ref[:bus])))

    ref[:ne_gen] = Dict(x for x in ref[:ne_gen] if (1 != pm_component_status_inactive["ne_gen"] && x.second["bus"] in keys(ref[:bus])))

    ref[:ne_storage] = Dict(x for x in ref[:ne_storage] if (1 != pm_component_status_inactive["ne_storage"] && x.second["storage_bus"] in keys(ref[:bus])))

    ref[:ne_arcs_from] = [(i,branch["f_bus"],branch["t_bus"]) for (i,branch) in ref[:ne_branch]]
    ref[:ne_arcs_to]   = [(i,branch["t_bus"],branch["f_bus"]) for (i,branch) in ref[:ne_branch]]
    ref[:ne_arcs] = [ref[:ne_arcs_from]; ref[:ne_arcs_to]]
    
    ne_bus_arcs = Dict((i, []) for (i,bus) in ref[:bus])
    for (l,i,j) in ref[:ne_arcs]
        push!(ne_bus_arcs[i], (l,i,j))
    end
    ref[:ne_bus_arcs] = ne_bus_arcs
    
    if !haskey(ref, :ne_buspairs)
        ref[:ne_buspairs] = calc_buspair_parameters(ref[:bus], ref[:ne_branch])
    end

    bus_ne_gens = Dict((i, Int[]) for (i,bus) in ref[:bus])
    for (i,gen) in ref[:ne_gen]
        push!(bus_ne_gens[gen["bus"]], i)
    end
    ref[:bus_ne_gens] = bus_ne_gens

    bus_ne_storage = Dict((i, Int[]) for (i,bus) in ref[:bus])
    for (i,strg) in ref[:ne_storage]
        push!(bus_ne_storage[strg["storage_bus"]], i)
    end
    ref[:bus_ne_storage] = bus_ne_storage

end

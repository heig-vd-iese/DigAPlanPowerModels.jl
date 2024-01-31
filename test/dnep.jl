function check_dnep_status(sol)
    for (idx,val) in sol["ne_branch"]
        @test isapprox(val["built"], 0.0, atol=1e-6, rtol=1e-6) || isapprox(val["built"], 1.0, atol=1e-6, rtol=1e-6)
    end
    for (idx,val) in sol["ne_gen"]
        @test isapprox(val["built"], 0.0, atol=1e-6, rtol=1e-6) || isapprox(val["built"], 1.0, atol=1e-6, rtol=1e-6)
    end
end

# test that ne_branch and ne_gen solution information is a superset of regular branches
function check_ne_keys(sol)
    branch = collect(sol["branch"])[1].second
    ne_brnach = collect(sol["ne_branch"])[1].second
    @test all(haskey(ne_brnach, k) for k in keys(branch))
    gen = collect(sol["gen"])[1].second
    ne_gen = collect(sol["ne_gen"])[1].second
    @test all(haskey(ne_gen, k) for k in keys(gen))
end


@testset "test ac dnep" begin
    @testset "3-bus case" begin
        data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
        calc_thermal_limits!(data)
        result = solve_dnep(data, ACPPowerModel, minlp_solver)

        check_dnep_status(result["solution"])
        check_ne_keys(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)
    end

    @testset "5-bus case" begin
        result = solve_dnep("../test/data/matpower/case5_dnep.m", ACPPowerModel, minlp_solver)

        check_dnep_status(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 1; atol = 1e-2)
    end
end


@testset "test soc dnep" begin
    @testset "3-bus case" begin
        data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
        calc_thermal_limits!(data)
        result = solve_dnep(data, SOCWRPowerModel, minlp_solver)

        check_dnep_status(result["solution"])
        check_ne_keys(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)
    end

    @testset "5-bus rts case" begin
        result = solve_dnep("../test/data/matpower/case5_dnep.m", SOCWRPowerModel, minlp_solver)

        check_dnep_status(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 1; atol = 1e-2)
    end
end


# requires a correct implementation of `variable_ne_branch_voltage`
# @testset "test qc dnep" begin
#     @testset "3-bus case" begin
#         data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
#         calc_thermal_limits!(data)
#         result = solve_dnep(data, QCRMPowerModel, minlp_solver)

#         check_dnep_status(result["solution"])
#         check_ne_branch_keys(result["solution"])

#         @test result["termination_status"] == LOCALLY_SOLVED
#         @test isapprox(result["objective"], 2; atol = 1e-2)
#     end

#     @testset "5-bus rts case" begin
#         result = solve_dnep("../test/data/matpower/case5_dnep.m", QCRMPowerModel, minlp_solver)

#         check_dnep_status(result["solution"])

#         @test result["termination_status"] == LOCALLY_SOLVED
#         @test isapprox(result["objective"], 1; atol = 1e-2)
#     end
# end


@testset "test dc dnep" begin
    @testset "3-bus case" begin
        data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
        calc_thermal_limits!(data)
        result = solve_dnep(data, DCPPowerModel, minlp_solver)

        check_dnep_status(result["solution"])
        check_ne_keys(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)
    end

    @testset "5-bus case" begin
        result = solve_dnep("../test/data/matpower/case5_dnep.m", DCPPowerModel, minlp_solver)

        check_dnep_status(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 1; atol = 1e-2)
    end
end


@testset "test matpower dc dnep" begin
    @testset "5-bus case with matpower DCMP model and DNEP" begin
        result = solve_dnep("../test/data/matpower/case5_dnep.m", DCMPPowerModel, minlp_solver)
        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["solution"]["ne_branch"]["1"]["built"], 1.0; atol = 1e-5)
        @test isapprox(result["solution"]["ne_branch"]["2"]["built"], 0.0; atol = 1e-5)
        @test isapprox(result["solution"]["ne_branch"]["3"]["built"], 0.0; atol = 1e-5)
        @test isapprox(result["solution"]["ne_gen"]["1"]["built"], 0.0; atol = 1e-5)
    end
end


@testset "test dc-losses dnep" begin
    #=
    # turn off due to numerical stability across operating systems
    @testset "3-bus case" begin
        data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
        calc_thermal_limits!(data)
        result = solve_dnep(data, DCPLLPowerModel, minlp_solver)

        check_dnep_status(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)
    end
    =#

    @testset "5-bus case" begin
        result = solve_dnep("../test/data/matpower/case5_dnep.m", DCPLLPowerModel, minlp_solver)

        check_dnep_status(result["solution"])
        check_ne_keys(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 1; atol = 1e-2)
    end
end

@testset "test lpac dnep" begin
    @testset "3-bus case" begin
        data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
        calc_thermal_limits!(data)
        result = solve_dnep(data, LPACCPowerModel, minlp_solver)

        check_dnep_status(result["solution"])
        check_ne_keys(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)
    end

    @testset "5-bus case" begin
        result = solve_dnep("../test/data/matpower/case5_dnep.m", LPACCPowerModel, minlp_solver)

        check_dnep_status(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 1; atol = 1e-2)
    end
end

@testset "test dnep branch flow output" begin
    @testset "3-bus case" begin
        data = PowerModels.parse_file("../test/data/matpower/case3_dnep.m")
        calc_thermal_limits!(data)
        result = solve_dnep(data, SOCWRPowerModel, minlp_solver)

        check_dnep_status(result["solution"])

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)

        branches = result["solution"]["branch"]
        ne_branches = result["solution"]["ne_branch"]
        flow_keys = ["pf","qf","pt","qt"]

        for fk in flow_keys
            @test !isnan(branches["1"][fk])
            @test !isnan(ne_branches["1"][fk])
            @test !isnan(ne_branches["2"][fk])
            @test !isnan(ne_branches["3"][fk])
        end
    end
end

TESTLOG = Memento.getlogger(PowerModels)
@testset "test denp multi networks" begin
    @testset "test solve_opf with multinetwork data" begin
        mn_data = build_mn_data("../test/data/matpower/case5_dnep_mn_strg.m")
        @test_throws(TESTLOG, ErrorException, PowerModels.solve_opf(mn_data, ACPPowerModel, minlp_solver))
    end

    @testset "test solve_mn_opf with single-network data" begin
        @test_throws(TESTLOG, ErrorException, PowerModels.solve_mn_opf("../test/data/matpower/case5_dnep_mn_strg.m", ACPPowerModel, nlp_solver))
    end

    @testset "test multi-network solution" begin
        # test case where generator status is 1 but the gen_bus status is 0
        mn_data = build_mn_data("../test/data/matpower/case5_dnep_mn_strg.m")
        result = PowerModels.solve_dnep_mn_strg(mn_data, ACPPowerModel, minlp_solver)

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 1.001; atol = 1e-3)

        @test InfrastructureModels.ismultinetwork(mn_data) == InfrastructureModels.ismultinetwork(result["solution"])
    end

    @testset "test make_multinetwork" begin
        data = PowerModels.parse_file("../test/data/matpower/case5_dnep_mn_strg.m")
        data["time_series"] = Dict(
            "num_steps" => 6,
            "load" => Dict(
                "1" => Dict("pd" => [3.0, 3.1, 2.9, 3.1, 3.2, 3.0]), # it should be in per-unit
                "2" => Dict("pd" => [3.0, 3.1, 2.9, 3.1, 3.2, 3.0]),
                "3" => Dict("pd" => [4.0, 4.1, 3.9, 4.2, 4.3, 4.1])
            )
        )

        mn_data = make_multinetwork(data)
        result = PowerModels.solve_dnep_mn_strg(mn_data, ACPPowerModel, minlp_solver)

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2.501; atol = 1e-3)

        @test InfrastructureModels.ismultinetwork(mn_data) == InfrastructureModels.ismultinetwork(result["solution"])
    end
end

@testset "test dnep" begin
    @testset "cigre" begin
        data = PowerModels.parse_file("../test/data/cigre.json")
        calc_thermal_limits!(data)
        result = solve_dnep(data, ACPPowerModel, minlp_solver)

        @test result["termination_status"] == LOCALLY_SOLVED
        @test isapprox(result["objective"], 2; atol = 1e-2)
    end
end
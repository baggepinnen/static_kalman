cd(@__DIR__)
using Pkg
Pkg.activate(".")
using ModelingToolkit, StaticArrays
p_cstr = (;
    K0_ab   = 1.287e12, # K0 [h^-1]
    K0_bc   = 1.287e12, # K0 [h^-1]
    K0_ad   = 9.043e9, # K0 [l/mol.h]
    R_gas   = 8.31446e-3, # Universal gas constant
    E_A_ab  = 9758.3, #* R_gas,# [kj/mol]
    E_A_bc  = 9758.3, #* R_gas,# [kj/mol]
    E_A_ad  = 8560.0, #* R_gas,# [kj/mol]
    Hᵣ_ab   = 4.2, # [kj/mol A]
    Hᵣ_bc   = -11.0, # [kj/mol B] Exothermic
    Hᵣ_ad   = -41.85, # [kj/mol A] Exothermic
    Rou     = 0.9342, # Density [kg/l]
    Cp      = 3.01, # Specific Heat capacity [kj/Kg.K]
    Cpₖ     = 2.0, # Coolant heat capacity [kj/kg.k]
    Aᵣ      = 0.215, # Area of reactor wall [m^2]
    Vᵣ      = 10.0, #0.01, # Volume of reactor [l]
    m_k     = 5.0, # Coolant mass[kg]
    T_in    = 130.0, # Temp of inflow [Celsius]
    K_w     = 4032.0, # [kj/h.m^2.K]
    C_A0    = (5.7+4.5)/2.0*1.0,  # Concentration of A in input Upper bound 5.7 lower bound 4.5 [mol/l]
)

function cstr(x, u, p=p_cstr, _=0)
    Cₐ, Cᵦ, Tᵣ, Tₖ = x

    (; K0_ab,K0_bc,K0_ad,R_gas,E_A_ab,E_A_bc,E_A_ad,Hᵣ_ab,Hᵣ_bc,Hᵣ_ad,Rou,Cp,Cpₖ,Aᵣ,Vᵣ,m_k,T_in,K_w,C_A0) = p


    F, Q̇    = u
    K₁      = K0_ab * exp((-E_A_ab)/((Tᵣ+273.15)))
    K₂      = K0_bc * exp((-E_A_bc)/((Tᵣ+273.15)))
    K₃      = K0_ad * exp((-E_A_ad)/((Tᵣ+273.15)))
    TΔ      = Tᵣ-Tₖ
    SA[
        F*(C_A0 - Cₐ)-K₁*Cₐ - K₃*abs2(Cₐ),
        -F*Cᵦ + K₁*Cₐ - K₂*Cᵦ,
        ((K₁*Cₐ*Hᵣ_ab + K₂*Cᵦ*Hᵣ_bc + K₃*abs2(Cₐ)*Hᵣ_ad)/(-Rou*Cp)) + F*(T_in-Tᵣ) + (((K_w*Aᵣ)*(-TΔ))/(Rou*Cp*Vᵣ)),
        (Q̇ + K_w*Aᵣ*(TΔ))/(m_k*Cpₖ)
    ]
end

"""
    sys = cstr()

A model of the continuously stirred tank reactor (CSTR).
This model has 4 states and 2 inputs.

`sys` has the following fields:
- `dynamics`: the dynamics of the system in the form of a [`FunctionSystem`](@ref)
- `sys`: the system in the form of a [`ODESystem`](@ref)
- `Ts`: a suggested sample time for the system
- `x0`: a suggested initial state of the system
- `lb`: a vector of pairs with lower bounds for all variables
- `ub`: a vector of pairs with upper bounds for all variables
- `p`: A NamedTuple with the default parameters of the system
"""
function CSTR(; name)
    @parameters t
    D = Differential(t)
    @variables begin
        Cₐ(t), [description = "Concentration of reactant A"],
        Cᵦ(t), [description = "Concentration of reactant B"],
        Tᵣ(t), [description = "Temperature in reactor"],
        Tₖ(t), [description = "Temperature in cooling jacket"],
        F(t), [description = "Feed", input=true],
        Q̇(t), [description = "Heat flow", input=true]
    end
    vars = [Cₐ, Cᵦ, Tᵣ, Tₖ, F, Q̇]
    eqs = cstr([Cₐ, Cᵦ, Tᵣ, Tₖ], [F, Q̇], p_cstr, t)
    sys = ODESystem(D.([Cₐ, Cᵦ, Tᵣ, Tₖ]) .~ eqs, t; name)
end

@named model = CSTR()
cmodel = complete(model)
inputs = [cmodel.F, cmodel.Q̇]
(f_oop, f_ip), x_sym, p, io_sys = ModelingToolkit.generate_control_function(model, inputs)


script = "dynamics.jl"
name,ext = splitext(script)



# oop
using MacroTools
expr = ModelingToolkit.RuntimeGeneratedFunctions.get_expression(f_oop.f_oop)
expr = expr |> MacroTools.flatten |> MacroTools.unblock |> MacroTools.rmlines # |> MacroTools.prettify

expr = MacroTools.postwalk(expr) do x
    if @capture(x, f_(args__)) && Symbol(f) == :create_array
        quote
            SA[$(args[5:end]...)]
        end
    elseif @capture(x, y_) && y isa LineNumberNode
        :()
    else
        x
    end
end
expr = expr |> MacroTools.flatten |> MacroTools.unblock |> MacroTools.rmlines
display(expr)

dict = splitdef(expr)
dict[:name] = Symbol(name)
expr = MacroTools.combinedef(dict)

open(script, "w") do io
    println(io, "\"\"\"")
    println(io, "$name(x, u, p, t)")
    println(io, "state x: ", string.(x_sym))
    println(io, "input u: ", string.(inputs))
    println(io, "parameters p: ", string.(p))
    println(io, "\"\"\"")
    println(io, expr)
end
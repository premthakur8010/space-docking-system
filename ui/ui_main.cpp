// ============================================================
// SPACE DOCKING SYSTEM: UI & VISUALIZATION LAYER (CLIENT)
// ============================================================
// Connects to the Physics Engine over UDP, renders 3D spacecraft poses
// with OpenGL 3.3 Core, and displays live telemetry via Dear ImGui.
// ============================================================

#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <cmath>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "common/math_types.hpp"
#include "common/protocol.hpp"
#include "common/net.hpp"

// ============================================================
// 1. SHADERS & MESH
// ============================================================
const char* vertexShaderSource = R"(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 FragPos;
out vec3 Normal;

void main()
{
    FragPos = vec3(uModel * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(uModel))) * aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 330 core

in vec3 FragPos;
in vec3 Normal;

uniform vec3 uColor;

out vec4 FragColor;

void main()
{
    vec3 lightDir = normalize(vec3(2.0, 3.0, 2.5));
    vec3 normal = normalize(Normal);

    float diffuse = max(dot(normal, lightDir), 0.0);

    vec3 ambient = 0.35 * uColor;
    vec3 diffuseColor = diffuse * uColor * 0.85;

    FragColor = vec4(ambient + diffuseColor, 1.0);
}
)";

static GLuint CompileShader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok)
    {
        char log[1024];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::cerr << "Shader compile error:\n" << log << "\n";
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

static GLuint CreateShaderProgram()
{
    GLuint vs = CompileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);
    if (!vs || !fs) return 0;

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok)
    {
        glDeleteProgram(prog);
        return 0;
    }
    return prog;
}

static void CreateCubeMesh(GLuint& vao, GLuint& vbo)
{
    float vertices[] = {
        // Front
        -0.5f,-0.5f,-0.5f, 0,0,-1,  0.5f,-0.5f,-0.5f, 0,0,-1,  0.5f, 0.5f,-0.5f, 0,0,-1,
         0.5f, 0.5f,-0.5f, 0,0,-1, -0.5f, 0.5f,-0.5f, 0,0,-1, -0.5f,-0.5f,-0.5f, 0,0,-1,
        // Back
        -0.5f,-0.5f, 0.5f, 0,0,1,   0.5f,-0.5f, 0.5f, 0,0,1,   0.5f, 0.5f, 0.5f, 0,0,1,
         0.5f, 0.5f, 0.5f, 0,0,1,  -0.5f, 0.5f, 0.5f, 0,0,1,  -0.5f,-0.5f, 0.5f, 0,0,1,
        // Left
        -0.5f, 0.5f, 0.5f,-1,0,0,  -0.5f, 0.5f,-0.5f,-1,0,0,  -0.5f,-0.5f,-0.5f,-1,0,0,
        -0.5f,-0.5f,-0.5f,-1,0,0,  -0.5f,-0.5f, 0.5f,-1,0,0,  -0.5f, 0.5f, 0.5f,-1,0,0,
        // Right
         0.5f, 0.5f, 0.5f, 1,0,0,   0.5f, 0.5f,-0.5f, 1,0,0,   0.5f,-0.5f,-0.5f, 1,0,0,
         0.5f,-0.5f,-0.5f, 1,0,0,   0.5f,-0.5f, 0.5f, 1,0,0,   0.5f, 0.5f, 0.5f, 1,0,0,
        // Bottom
        -0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f,-0.5f, 0,-1,0,  0.5f,-0.5f, 0.5f, 0,-1,0,
         0.5f,-0.5f, 0.5f, 0,-1,0, -0.5f,-0.5f, 0.5f, 0,-1,0, -0.5f,-0.5f,-0.5f, 0,-1,0,
        // Top
        -0.5f, 0.5f,-0.5f, 0,1,0,   0.5f, 0.5f,-0.5f, 0,1,0,   0.5f, 0.5f, 0.5f, 0,1,0,
         0.5f, 0.5f, 0.5f, 0,1,0,  -0.5f, 0.5f, 0.5f, 0,1,0,  -0.5f, 0.5f,-0.5f, 0,1,0
    };

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

static void RenderCube(
    GLuint shaderProgram,
    const Vec3& position,
    const Quat& orientation,
    const Vec3& size,
    const glm::mat4& projection,
    const glm::mat4& view,
    float r, float g, float b)
{
    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(
        static_cast<float>(position.x),
        static_cast<float>(position.y),
        static_cast<float>(position.z)
    ));

    model = model * orientation.ToRotationMatrix();

    model = glm::scale(model, glm::vec3(
        static_cast<float>(size.x),
        static_cast<float>(size.y),
        static_cast<float>(size.z)
    ));

    glm::mat4 mvp = projection * view * model;

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(mvp));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), r, g, b);

    glDrawArrays(GL_TRIANGLES, 0, 36);
}

// ============================================================
// 2. IMGUI STYLING
// ============================================================
static void SetupImGuiStyle()
{
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowPadding     = ImVec2(20.0f, 18.0f);
    style.WindowRounding    = 12.0f;
    style.ChildRounding     = 8.0f;
    style.FrameRounding     = 6.0f;
    style.PopupRounding     = 8.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding      = 4.0f;
    style.TabRounding       = 6.0f;

    style.FramePadding      = ImVec2(10.0f, 6.0f);
    style.ItemSpacing       = ImVec2(10.0f, 8.0f);
    style.ItemInnerSpacing  = ImVec2(8.0f, 5.0f);
    style.WindowBorderSize  = 1.0f;
    style.FrameBorderSize   = 0.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text]                  = ImVec4(0.93f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled]          = ImVec4(0.55f, 0.62f, 0.72f, 1.00f);
    colors[ImGuiCol_WindowBg]              = ImVec4(0.08f, 0.10f, 0.14f, 0.88f);
    colors[ImGuiCol_ChildBg]               = ImVec4(0.11f, 0.14f, 0.20f, 0.50f);
    colors[ImGuiCol_PopupBg]               = ImVec4(0.10f, 0.12f, 0.17f, 0.95f);
    colors[ImGuiCol_Border]                = ImVec4(0.24f, 0.32f, 0.46f, 0.65f);
    colors[ImGuiCol_BorderShadow]          = ImVec4(0.00f, 0.00f, 0.00f, 0.40f);
    colors[ImGuiCol_FrameBg]               = ImVec4(0.14f, 0.18f, 0.26f, 0.70f);
    colors[ImGuiCol_FrameBgHovered]        = ImVec4(0.20f, 0.26f, 0.38f, 0.85f);
    colors[ImGuiCol_FrameBgActive]         = ImVec4(0.25f, 0.32f, 0.46f, 0.95f);
    colors[ImGuiCol_TitleBg]               = ImVec4(0.10f, 0.13f, 0.19f, 1.00f);
    colors[ImGuiCol_TitleBgActive]         = ImVec4(0.14f, 0.19f, 0.28f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]      = ImVec4(0.08f, 0.10f, 0.14f, 0.75f);
    colors[ImGuiCol_Button]                = ImVec4(0.16f, 0.22f, 0.32f, 0.85f);
    colors[ImGuiCol_ButtonHovered]         = ImVec4(0.24f, 0.34f, 0.48f, 0.95f);
    colors[ImGuiCol_ButtonActive]          = ImVec4(0.30f, 0.42f, 0.60f, 1.00f);
    colors[ImGuiCol_Separator]             = ImVec4(0.22f, 0.30f, 0.42f, 0.70f);
}

// ============================================================
// 3. GLOBAL IPC & UI STATE
// ============================================================
struct UIConfig
{
    std::string physicsHost = SIM_DEFAULT_HOST;
    int statePort = SIM_STATE_PORT;
    int cmdPort   = SIM_COMMAND_PORT;
};

UIConfig g_config;
UdpSocket g_stateSocket;
UdpSocket g_cmdSocket;

SimulationPacket g_latestPkt{};
bool g_haveReceivedPacket = false;
std::chrono::steady_clock::time_point g_lastPacketTime;
std::chrono::steady_clock::time_point g_uiStartTime;

static void SendCommand(SimCommand cmd)
{
    CommandPacket pkt{};
    pkt.magic = CMD_PACKET_MAGIC;
    pkt.cmd = cmd;
    g_cmdSocket.SendTo(&pkt, sizeof(pkt), g_config.physicsHost.c_str(), g_config.cmdPort);
}

static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    (void)window;
    (void)scancode;
    (void)mods;
    if (action != GLFW_PRESS)
        return;

    if (key == GLFW_KEY_1)
    {
        std::cout << "[UI] Sent Command: Scenario 1 (Head-On)\n";
        SendCommand(SimCommand::Scenario1);
    }
    else if (key == GLFW_KEY_2)
    {
        std::cout << "[UI] Sent Command: Scenario 2 (Off-Center)\n";
        SendCommand(SimCommand::Scenario2);
    }
    else if (key == GLFW_KEY_R)
    {
        std::cout << "[UI] Sent Command: Reset\n";
        SendCommand(SimCommand::Reset);
    }
    else if (key == GLFW_KEY_SPACE)
    {
        std::cout << "[UI] Sent Command: Toggle Pause\n";
        SendCommand(SimCommand::TogglePause);
    }
}

static void DrawCubeBlock(const char* title, const Vec3& p, const Quat& q)
{
    ImGui::Text("%s", title);
    ImGui::Separator();
    ImGui::Text("Position");
    ImGui::Text("  X: % .6f", p.x);
    ImGui::Text("  Y: % .6f", p.y);
    ImGui::Text("  Z: % .6f", p.z);
    ImGui::Spacing();
    ImGui::Text("Quaternion");
    ImGui::Text("  W: % .6f", q.w);
    ImGui::Text("  X: % .6f", q.x);
    ImGui::Text("  Y: % .6f", q.y);
    ImGui::Text("  Z: % .6f", q.z);
}

static void RenderSimulationText(bool connected)
{
    float displayWidth = ImGui::GetIO().DisplaySize.x;
    float displayHeight = ImGui::GetIO().DisplaySize.y;

    float panelWidth = std::clamp(displayWidth * 0.35f, 520.0f, 720.0f);
    float panelHeight = displayHeight - 24.0f;

    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panelWidth, panelHeight), ImGuiCond_Always);

    ImGui::Begin(
        "Simulation State",
        nullptr,
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse
    );

    // Connection Status Banner
    if (connected)
    {
        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.4f, 1.0f), "[STATUS: CONNECTED TO PHYSICS ENGINE]");
        if (g_latestPkt.paused)
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), ">>> SIMULATION PAUSED <<<");
    }
    else
    {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[STATUS: DISCONNECTED / WAITING FOR PHYSICS]");
        ImGui::TextDisabled("Listening on port %d...", g_config.statePort);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    Vec3 tPos(g_latestPkt.targetPos[0], g_latestPkt.targetPos[1], g_latestPkt.targetPos[2]);
    Quat tQuat(g_latestPkt.targetQuat[0], g_latestPkt.targetQuat[1], g_latestPkt.targetQuat[2], g_latestPkt.targetQuat[3]);
    DrawCubeBlock("CUBE 1 - TARGET", tPos, tQuat);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    Vec3 cPos(g_latestPkt.chaserPos[0], g_latestPkt.chaserPos[1], g_latestPkt.chaserPos[2]);
    Quat cQuat(g_latestPkt.chaserQuat[0], g_latestPkt.chaserQuat[1], g_latestPkt.chaserQuat[2], g_latestPkt.chaserQuat[3]);
    DrawCubeBlock("CUBE 2 - CHASER", cPos, cQuat);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("Physics Telemetry");
    ImGui::Separator();
    ImGui::Text("  Step: %llu", static_cast<unsigned long long>(g_latestPkt.step));

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - g_uiStartTime).count();

    if (g_latestPkt.secondsSpecified)
    {
        const double remain = std::max(0.0, g_latestPkt.headlessSeconds - elapsed);
        ImGui::Text("  Timer: %.1f / %.0f s (%.1f s left)", elapsed, g_latestPkt.headlessSeconds, remain);
    }
    else
    {
        ImGui::Text("  Elapsed: %.1f s", elapsed);
    }

    int periodUs = g_latestPkt.physicsPeriodUs > 0 ? g_latestPkt.physicsPeriodUs : 1000;
    ImGui::Text("  Frequency: %.0f Hz", 1e6 / periodUs);
    ImGui::Text("  Timestep: %d us", periodUs);

    if (connected)
    {
        ImGui::Text("  Avg period: %.1f us", g_latestPkt.avgPeriodNs / 1000.0);
        ImGui::Text("  Max period: %.1f us", g_latestPkt.maxPeriodNs / 1000.0);
        ImGui::Text("  Max work: %.1f us (< 1 ms)", g_latestPkt.maxWorkNs / 1000.0);
        ImGui::Text("  Max start latency: %.1f us", g_latestPkt.maxLatencyNs / 1000.0);
        ImGui::Text("  Ticks > %.0f us late: %llu", g_latestPkt.tolUs,
                    static_cast<unsigned long long>(g_latestPkt.latencyViolations));
        ImGui::Text("  Late ticks: %llu", static_cast<unsigned long long>(g_latestPkt.lateTicks));
    }
    else
    {
        ImGui::Text("  Avg period: --");
        ImGui::Text("  Max period: --");
        ImGui::Text("  Max work: --");
        ImGui::Text("  Late ticks: --");
    }

    ImGui::Text("  Rendering: ~60 FPS");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextDisabled("Controls: 1/2 scenario, R reset, SPACE pause");

    ImGui::End();
}

// ============================================================
// 4. MAIN UI CLIENT ENTRY POINT
// ============================================================
int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--host" && i + 1 < argc) g_config.physicsHost = argv[++i];
        else if (a == "--state-port" && i + 1 < argc) g_config.statePort = std::atoi(argv[++i]);
        else if (a == "--cmd-port" && i + 1 < argc) g_config.cmdPort = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --host IP        physics server IP (default 127.0.0.1)\n"
                      << "  --state-port P   port to listen for simulation state (default 9870)\n"
                      << "  --cmd-port P     port to send commands to physics (default 9871)\n";
            return 0;
        }
    }

    std::cout
        << "====================================================\n"
        << " SPACE DOCKING UI CLIENT\n"
        << "====================================================\n"
        << "Listening state  : UDP <- 127.0.0.1:" << g_config.statePort << "\n"
        << "Sending commands : UDP -> " << g_config.physicsHost << ":" << g_config.cmdPort << "\n"
        << "Rendering        : ~60 FPS OpenGL 3.3 Core\n"
        << "Keys             : 1 / 2 scenario, R reset, SPACE pause\n"
        << "====================================================\n";

    // Bind state listening socket on 0.0.0.0 (or 127.0.0.1)
    if (!g_stateSocket.Bind(g_config.statePort, "0.0.0.0"))
    {
        std::cerr << "ERROR: Could not bind state port " << g_config.statePort
                  << " (already in use?)\n";
        return -1;
    }
    g_stateSocket.SetNonBlocking(true);

    // Open command sending socket
    g_cmdSocket.Open();

    // Default starting poses in case physics hasn't started yet
    g_latestPkt.targetPos[0] = 0.0; g_latestPkt.targetPos[1] = 0.0; g_latestPkt.targetPos[2] = 0.0;
    g_latestPkt.targetQuat[0] = 1.0;
    g_latestPkt.chaserPos[0] = -3.0; g_latestPkt.chaserPos[1] = 0.0; g_latestPkt.chaserPos[2] = 0.0;
    g_latestPkt.chaserQuat[0] = 1.0;
    g_latestPkt.physicsPeriodUs = 1000;

    // Window and OpenGL
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(1100, 700, "Spacecraft Docking UI", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, KeyCallback);

    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
    {
        std::cerr << "Failed to initialize GLAD\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    GLuint shaderProgram = CreateShaderProgram();
    if (!shaderProgram)
    {
        std::cerr << "Failed to create shader program\n";
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    GLuint cubeVAO, cubeVBO;
    CreateCubeMesh(cubeVAO, cubeVBO);

    // ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.FontGlobalScale = 2.0f;

    SetupImGuiStyle();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    g_uiStartTime = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        // 1. Drain UDP telemetry socket to get latest packet
        SimulationPacket incoming;
        bool gotAny = false;
        while (g_stateSocket.RecvFrom(&incoming, sizeof(incoming)) == sizeof(incoming))
        {
            if (incoming.magic == SIM_PACKET_MAGIC)
            {
                g_latestPkt = incoming;
                gotAny = true;
            }
        }

        if (gotAny)
        {
            g_haveReceivedPacket = true;
            g_lastPacketTime = std::chrono::steady_clock::now();
        }

        // Connection considered active if packet received within last 1.5 seconds
        const bool connected = g_haveReceivedPacket &&
            (std::chrono::duration<double>(std::chrono::steady_clock::now() - g_lastPacketTime).count() < 1.5);

        int windowWidth = 0;
        int windowHeight = 0;
        glfwGetFramebufferSize(window, &windowWidth, &windowHeight);

        if (windowWidth == 0 || windowHeight == 0)
        {
            glfwWaitEvents();
            continue;
        }

        glViewport(0, 0, windowWidth, windowHeight);

        // Dynamically scale font
        ImGuiIO& io = ImGui::GetIO();
        if (windowHeight >= 1000)
            io.FontGlobalScale = 2.40f;
        else if (windowHeight >= 850)
            io.FontGlobalScale = 2.10f;
        else
            io.FontGlobalScale = 1.90f;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        glClearColor(0.12f, 0.14f, 0.18f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);

        float aspectRatio = static_cast<float>(windowWidth) / static_cast<float>(windowHeight);
        glm::mat4 projection = glm::perspective(glm::radians(42.0f), aspectRatio, 0.1f, 50.0f);
        glm::mat4 view = glm::lookAt(
            glm::vec3(0.0f, 3.8f, 6.8f),
            glm::vec3(0.0f, 0.2f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f)
        );

        glBindVertexArray(cubeVAO);

        // Target Cube (Cyan/Blue)
        Vec3 tPos(g_latestPkt.targetPos[0], g_latestPkt.targetPos[1], g_latestPkt.targetPos[2]);
        Quat tQuat(g_latestPkt.targetQuat[0], g_latestPkt.targetQuat[1], g_latestPkt.targetQuat[2], g_latestPkt.targetQuat[3]);
        RenderCube(shaderProgram, tPos, tQuat, Vec3(1.0, 1.0, 1.0), projection, view, 0.25f, 0.70f, 0.90f);

        // Chaser Cube (Orange)
        Vec3 cPos(g_latestPkt.chaserPos[0], g_latestPkt.chaserPos[1], g_latestPkt.chaserPos[2]);
        Quat cQuat(g_latestPkt.chaserQuat[0], g_latestPkt.chaserQuat[1], g_latestPkt.chaserQuat[2], g_latestPkt.chaserQuat[3]);
        RenderCube(shaderProgram, cPos, cQuat, Vec3(1.0, 1.0, 1.0), projection, view, 0.95f, 0.45f, 0.25f);

        // Floor
        glm::mat4 floorModel = glm::mat4(1.0f);
        floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.65f, 0.0f));
        floorModel = glm::scale(floorModel, glm::vec3(12.0f, 0.02f, 3.0f));
        glm::mat4 floorMVP = projection * view * floorModel;

        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(floorMVP));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(floorModel));
        glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.18f, 0.20f, 0.25f);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glBindVertexArray(0);

        // ImGui Overlay
        RenderSimulationText(connected);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

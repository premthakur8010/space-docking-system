// ============================================================
// SPACE DOCKING SYSTEM: UI & VISUALIZATION LAYER (CLIENT)
// ============================================================
// Connects to the Physics Engine over UDP Multicast, renders 3D spacecraft poses
// (Cube, Sphere, Cone, Pyramid) with OpenGL 3.3 Core at 90 FPS,
// and displays live telemetry via Dear ImGui.
// Multiple UI windows can run independently without port collisions.
// ============================================================

#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <thread>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
#endif

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
// 1. SHAPES & MESH GENERATION
// ============================================================
enum class BodyShape : int
{
    Cube    = 0,
    Sphere  = 1,
    Cone    = 2,
    Pyramid = 3
};

struct MeshData
{
    GLuint vao = 0;
    GLuint vbo = 0;
    GLsizei vertexCount = 0;
};

static MeshData g_shapeMeshes[4];

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

// --- Cube Mesh (36 vertices) ---
static void CreateCubeMesh(MeshData& mesh)
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

    mesh.vertexCount = 36;
    glGenVertexArrays(1, &mesh.vao);
    glGenBuffers(1, &mesh.vbo);
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

// --- Sphere Mesh (Smooth UV Sphere) ---
static void CreateSphereMesh(MeshData& mesh, int rings = 24, int sectors = 32)
{
    std::vector<float> vertices;
    const float radius = 0.5f;

    for (int r = 0; r < rings - 1; ++r)
    {
        for (int s = 0; s < sectors - 1; ++s)
        {
            float theta0 = s * 2.0f * static_cast<float>(M_PI) / (sectors - 1);
            float theta1 = (s + 1) * 2.0f * static_cast<float>(M_PI) / (sectors - 1);
            float phi0 = static_cast<float>(M_PI) * (-0.5f + static_cast<float>(r) / (rings - 1));
            float phi1 = static_cast<float>(M_PI) * (-0.5f + static_cast<float>(r + 1) / (rings - 1));

            auto addVert = [&](float theta, float phi) {
                float x = std::cos(phi) * std::sin(theta);
                float y = std::sin(phi);
                float z = std::cos(phi) * std::cos(theta);
                vertices.push_back(x * radius);
                vertices.push_back(y * radius);
                vertices.push_back(z * radius);
                vertices.push_back(x);
                vertices.push_back(y);
                vertices.push_back(z);
            };

            addVert(theta0, phi0);
            addVert(theta0, phi1);
            addVert(theta1, phi1);

            addVert(theta0, phi0);
            addVert(theta1, phi1);
            addVert(theta1, phi0);
        }
    }

    mesh.vertexCount = static_cast<GLsizei>(vertices.size() / 6);
    glGenVertexArrays(1, &mesh.vao);
    glGenBuffers(1, &mesh.vbo);
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

// --- Cone Mesh (Circular Base + Slanted Sides) ---
static void CreateConeMesh(MeshData& mesh, int segments = 32)
{
    std::vector<float> vertices;
    const float radius = 0.5f;
    const float yApex = 0.5f;
    const float yBase = -0.5f;
    const float slantNormY = radius / 1.0f; // slope for outward normal

    for (int i = 0; i < segments; ++i)
    {
        float theta0 = i * 2.0f * static_cast<float>(M_PI) / segments;
        float theta1 = (i + 1) * 2.0f * static_cast<float>(M_PI) / segments;

        float c0 = std::cos(theta0), s0 = std::sin(theta0);
        float c1 = std::cos(theta1), s1 = std::sin(theta1);

        glm::vec3 n0 = glm::normalize(glm::vec3(c0, slantNormY, s0));
        glm::vec3 n1 = glm::normalize(glm::vec3(c1, slantNormY, s1));
        glm::vec3 nApex = glm::normalize(glm::vec3(0.5f * (c0 + c1), slantNormY, 0.5f * (s0 + s1)));

        // Side triangle: Apex, Base1, Base2
        vertices.push_back(0.0f); vertices.push_back(yApex); vertices.push_back(0.0f);
        vertices.push_back(nApex.x); vertices.push_back(nApex.y); vertices.push_back(nApex.z);

        vertices.push_back(c0 * radius); vertices.push_back(yBase); vertices.push_back(s0 * radius);
        vertices.push_back(n0.x); vertices.push_back(n0.y); vertices.push_back(n0.z);

        vertices.push_back(c1 * radius); vertices.push_back(yBase); vertices.push_back(s1 * radius);
        vertices.push_back(n1.x); vertices.push_back(n1.y); vertices.push_back(n1.z);

        // Bottom cap triangle: Center, Base2, Base1 (facing down)
        vertices.push_back(0.0f); vertices.push_back(yBase); vertices.push_back(0.0f);
        vertices.push_back(0.0f); vertices.push_back(-1.0f); vertices.push_back(0.0f);

        vertices.push_back(c1 * radius); vertices.push_back(yBase); vertices.push_back(s1 * radius);
        vertices.push_back(0.0f); vertices.push_back(-1.0f); vertices.push_back(0.0f);

        vertices.push_back(c0 * radius); vertices.push_back(yBase); vertices.push_back(s0 * radius);
        vertices.push_back(0.0f); vertices.push_back(-1.0f); vertices.push_back(0.0f);
    }

    mesh.vertexCount = static_cast<GLsizei>(vertices.size() / 6);
    glGenVertexArrays(1, &mesh.vao);
    glGenBuffers(1, &mesh.vbo);
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

// --- Pyramid Mesh (Square Base + 4 Triangular Faces) ---
static void CreatePyramidMesh(MeshData& mesh)
{
    glm::vec3 apex(0.0f, 0.5f, 0.0f);
    glm::vec3 bl(-0.5f, -0.5f, -0.5f);
    glm::vec3 br( 0.5f, -0.5f, -0.5f);
    glm::vec3 fr( 0.5f, -0.5f,  0.5f);
    glm::vec3 fl(-0.5f, -0.5f,  0.5f);

    glm::vec3 nFront = glm::normalize(glm::cross(fr - apex, fl - apex));
    glm::vec3 nBack  = glm::normalize(glm::cross(bl - apex, br - apex));
    glm::vec3 nLeft  = glm::normalize(glm::cross(fl - apex, bl - apex));
    glm::vec3 nRight = glm::normalize(glm::cross(br - apex, fr - apex));
    glm::vec3 nBottom(0.0f, -1.0f, 0.0f);

    std::vector<float> vertices;
    auto addTri = [&](const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& n) {
        vertices.push_back(p1.x); vertices.push_back(p1.y); vertices.push_back(p1.z);
        vertices.push_back(n.x);  vertices.push_back(n.y);  vertices.push_back(n.z);
        vertices.push_back(p2.x); vertices.push_back(p2.y); vertices.push_back(p2.z);
        vertices.push_back(n.x);  vertices.push_back(n.y);  vertices.push_back(n.z);
        vertices.push_back(p3.x); vertices.push_back(p3.y); vertices.push_back(p3.z);
        vertices.push_back(n.x);  vertices.push_back(n.y);  vertices.push_back(n.z);
    };

    // 4 triangular sides
    addTri(apex, fl, fr, nFront);
    addTri(apex, br, bl, nBack);
    addTri(apex, bl, fl, nLeft);
    addTri(apex, fr, br, nRight);
    // Square base (2 triangles)
    addTri(bl, br, fr, nBottom);
    addTri(bl, fr, fl, nBottom);

    mesh.vertexCount = static_cast<GLsizei>(vertices.size() / 6);
    glGenVertexArrays(1, &mesh.vao);
    glGenBuffers(1, &mesh.vbo);
    glBindVertexArray(mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

// --- Unified Shape Rendering ---
static void RenderShape(
    GLuint shaderProgram,
    BodyShape shape,
    const Vec3& position,
    const Quat& orientation,
    const Vec3& size,
    const glm::mat4& projection,
    const glm::mat4& view,
    float r, float g, float b)
{
    int idx = static_cast<int>(shape);
    if (idx < 0 || idx >= 4) idx = 0;
    const MeshData& mesh = g_shapeMeshes[idx];

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

    glBindVertexArray(mesh.vao);
    glDrawArrays(GL_TRIANGLES, 0, mesh.vertexCount);
    glBindVertexArray(0);
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
    std::string physicsHost     = SIM_DEFAULT_HOST;
    std::string multicastGroup  = SIM_DEFAULT_MULTICAST_GROUP;
    int statePort               = SIM_STATE_PORT;
    int cmdPort                 = SIM_COMMAND_PORT;
    BodyShape shape             = BodyShape::Cube;
    std::string windowTitle     = "";
    int targetFps               = 90;
    int posX                    = -1;
    int posY                    = -1;
    int width                   = 960;
    int height                  = 640;
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

static void DrawPoseBlock(const char* title, const Vec3& p, const Quat& q)
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

    float panelWidth = std::clamp(displayWidth * 0.35f, 480.0f, 680.0f);
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
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[STATUS: WAITING FOR PHYSICS BROADCAST]");
        ImGui::TextDisabled("Listening on %s:%d...", g_config.multicastGroup.c_str(), g_config.statePort);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Shape Selector Radio Buttons
    ImGui::Text("ACTIVE VISUALIZATION SHAPE");
    int currentShape = static_cast<int>(g_config.shape);
    if (ImGui::RadioButton("Cube", &currentShape, 0))    g_config.shape = BodyShape::Cube;
    ImGui::SameLine();
    if (ImGui::RadioButton("Sphere", &currentShape, 1))  g_config.shape = BodyShape::Sphere;
    ImGui::SameLine();
    if (ImGui::RadioButton("Cone", &currentShape, 2))    g_config.shape = BodyShape::Cone;
    ImGui::SameLine();
    if (ImGui::RadioButton("Pyramid", &currentShape, 3)) g_config.shape = BodyShape::Pyramid;

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    const char* shapeNames[] = { "CUBE", "SPHERE", "CONE", "PYRAMID" };
    const char* sName = shapeNames[static_cast<int>(g_config.shape)];

    std::string tTitle = std::string(sName) + " 1 - TARGET";
    Vec3 tPos(g_latestPkt.targetPos[0], g_latestPkt.targetPos[1], g_latestPkt.targetPos[2]);
    Quat tQuat(g_latestPkt.targetQuat[0], g_latestPkt.targetQuat[1], g_latestPkt.targetQuat[2], g_latestPkt.targetQuat[3]);
    DrawPoseBlock(tTitle.c_str(), tPos, tQuat);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    std::string cTitle = std::string(sName) + " 2 - CHASER";
    Vec3 cPos(g_latestPkt.chaserPos[0], g_latestPkt.chaserPos[1], g_latestPkt.chaserPos[2]);
    Quat cQuat(g_latestPkt.chaserQuat[0], g_latestPkt.chaserQuat[1], g_latestPkt.chaserQuat[2], g_latestPkt.chaserQuat[3]);
    DrawPoseBlock(cTitle.c_str(), cPos, cQuat);

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
    ImGui::Text("  Physics Tick: %d us (%.0f Hz)", periodUs, 1e6 / periodUs);

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

    ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f),
        "  UI Render: %.1f FPS (Target: %d FPS)", ImGui::GetIO().Framerate, g_config.targetFps);

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
        else if (a == "--mcast" && i + 1 < argc) g_config.multicastGroup = argv[++i];
        else if (a == "--state-port" && i + 1 < argc) g_config.statePort = std::atoi(argv[++i]);
        else if (a == "--cmd-port" && i + 1 < argc) g_config.cmdPort = std::atoi(argv[++i]);
        else if ((a == "--shape" || a == "-s") && i + 1 < argc)
        {
            std::string s = argv[++i];
            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
            if (s == "cube" || s == "0") g_config.shape = BodyShape::Cube;
            else if (s == "sphere" || s == "1") g_config.shape = BodyShape::Sphere;
            else if (s == "cone" || s == "2") g_config.shape = BodyShape::Cone;
            else if (s == "pyramid" || s == "3") g_config.shape = BodyShape::Pyramid;
            else std::cerr << "[WARN] Unknown shape: " << s << " (defaulting to cube)\n";
        }
        else if (a == "--fps" && i + 1 < argc) g_config.targetFps = std::max(10, std::atoi(argv[++i]));
        else if (a == "--title" && i + 1 < argc) g_config.windowTitle = argv[++i];
        else if (a == "--pos-x" && i + 1 < argc) g_config.posX = std::atoi(argv[++i]);
        else if (a == "--pos-y" && i + 1 < argc) g_config.posY = std::atoi(argv[++i]);
        else if (a == "--width" && i + 1 < argc) g_config.width = std::atoi(argv[++i]);
        else if (a == "--height" && i + 1 < argc) g_config.height = std::atoi(argv[++i]);
        else if (a == "-h" || a == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --shape, -s TYPE shape to render: cube, sphere, cone, pyramid (default: cube)\n"
                      << "  --fps N          target render framerate (default: 90)\n"
                      << "  --title NAME     window title\n"
                      << "  --pos-x X        initial window X coordinate\n"
                      << "  --pos-y Y        initial window Y coordinate\n"
                      << "  --width W        window width (default: 960)\n"
                      << "  --height H       window height (default: 640)\n"
                      << "  --host IP        physics server IP for commands (default 127.0.0.1)\n"
                      << "  --mcast IP       multicast telemetry group (default 239.255.0.1)\n"
                      << "  --state-port P   port to listen for state (default 9870)\n"
                      << "  --cmd-port P     port to send commands (default 9871)\n";
            return 0;
        }
    }

    if (g_config.windowTitle.empty())
    {
        switch (g_config.shape)
        {
            case BodyShape::Cube:    g_config.windowTitle = "Spacecraft Docking - [CUBE]"; break;
            case BodyShape::Sphere:  g_config.windowTitle = "Spacecraft Docking - [SPHERE]"; break;
            case BodyShape::Cone:    g_config.windowTitle = "Spacecraft Docking - [CONE]"; break;
            case BodyShape::Pyramid: g_config.windowTitle = "Spacecraft Docking - [PYRAMID]"; break;
        }
    }

    std::cout
        << "====================================================\n"
        << " SPACE DOCKING UI CLIENT: " << g_config.windowTitle << "\n"
        << "====================================================\n"
        << "Listening stream : UDP Multicast " << g_config.multicastGroup << ":" << g_config.statePort << "\n"
        << "Sending commands : UDP -> " << g_config.physicsHost << ":" << g_config.cmdPort << "\n"
        << "Target Framerate : " << g_config.targetFps << " FPS\n"
        << "Shape Mode       : " << static_cast<int>(g_config.shape) << "\n"
        << "====================================================\n";

    // Bind state listening socket on 0.0.0.0 with SO_REUSEADDR and join multicast
    g_stateSocket.Open();
    g_stateSocket.SetReuseAddr(true);
    if (!g_stateSocket.Bind(g_config.statePort, "0.0.0.0"))
    {
        std::cerr << "ERROR: Could not bind state port " << g_config.statePort << "\n";
        return -1;
    }
    if (!g_stateSocket.JoinMulticastGroup(g_config.multicastGroup.c_str()))
    {
        std::cerr << "[WARN] Could not join multicast group " << g_config.multicastGroup
                  << " (continuing with local unicast/broadcast)\n";
    }
    g_stateSocket.SetNonBlocking(true);

    // Open command sending socket (ephemeral outgoing port)
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

    GLFWwindow* window = glfwCreateWindow(
        g_config.width, g_config.height,
        g_config.windowTitle.c_str(), nullptr, nullptr);

    if (!window)
    {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    if (g_config.posX >= 0 && g_config.posY >= 0)
    {
        glfwSetWindowPos(window, g_config.posX, g_config.posY);
    }

    glfwMakeContextCurrent(window);
    // Disable VSync to achieve exact 90 FPS independently
    glfwSwapInterval(0);
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

    // Generate all 4 shape meshes
    CreateCubeMesh(g_shapeMeshes[static_cast<int>(BodyShape::Cube)]);
    CreateSphereMesh(g_shapeMeshes[static_cast<int>(BodyShape::Sphere)]);
    CreateConeMesh(g_shapeMeshes[static_cast<int>(BodyShape::Cone)]);
    CreatePyramidMesh(g_shapeMeshes[static_cast<int>(BodyShape::Pyramid)]);

    // ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.FontGlobalScale = 1.8f;

    SetupImGuiStyle();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    g_uiStartTime = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        auto frameStartTime = std::chrono::steady_clock::now();

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
        if (windowHeight >= 1000)
            io.FontGlobalScale = 2.20f;
        else if (windowHeight >= 850)
            io.FontGlobalScale = 1.95f;
        else
            io.FontGlobalScale = 1.70f;

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

        // Target Body (Cyan / Blue)
        Vec3 tPos(g_latestPkt.targetPos[0], g_latestPkt.targetPos[1], g_latestPkt.targetPos[2]);
        Quat tQuat(g_latestPkt.targetQuat[0], g_latestPkt.targetQuat[1], g_latestPkt.targetQuat[2], g_latestPkt.targetQuat[3]);
        RenderShape(shaderProgram, g_config.shape, tPos, tQuat, Vec3(1.0, 1.0, 1.0), projection, view, 0.25f, 0.70f, 0.90f);

        // Chaser Body (Orange)
        Vec3 cPos(g_latestPkt.chaserPos[0], g_latestPkt.chaserPos[1], g_latestPkt.chaserPos[2]);
        Quat cQuat(g_latestPkt.chaserQuat[0], g_latestPkt.chaserQuat[1], g_latestPkt.chaserQuat[2], g_latestPkt.chaserQuat[3]);
        RenderShape(shaderProgram, g_config.shape, cPos, cQuat, Vec3(1.0, 1.0, 1.0), projection, view, 0.95f, 0.45f, 0.25f);

        // Floor (Rendered using Cube VAO)
        glm::mat4 floorModel = glm::mat4(1.0f);
        floorModel = glm::translate(floorModel, glm::vec3(0.0f, -0.65f, 0.0f));
        floorModel = glm::scale(floorModel, glm::vec3(12.0f, 0.02f, 3.0f));
        glm::mat4 floorMVP = projection * view * floorModel;

        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uMVP"), 1, GL_FALSE, glm::value_ptr(floorMVP));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(floorModel));
        glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.18f, 0.20f, 0.25f);

        glBindVertexArray(g_shapeMeshes[static_cast<int>(BodyShape::Cube)].vao);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);

        // ImGui Overlay
        RenderSimulationText(connected);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();

        // High-precision 90 FPS limiter
        const int64_t targetFrameUs = 1000000 / g_config.targetFps;
        auto frameEndTime = std::chrono::steady_clock::now();
        auto frameDurationUs = std::chrono::duration_cast<std::chrono::microseconds>(frameEndTime - frameStartTime).count();

        if (frameDurationUs < targetFrameUs)
        {
            int64_t sleepUs = targetFrameUs - frameDurationUs;
            if (sleepUs > 1500)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(sleepUs - 1000));
            }
            while (std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - frameStartTime).count() < targetFrameUs)
            {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
                _mm_pause();
#endif
            }
        }
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    for (int i = 0; i < 4; ++i)
    {
        if (g_shapeMeshes[i].vao) glDeleteVertexArrays(1, &g_shapeMeshes[i].vao);
        if (g_shapeMeshes[i].vbo) glDeleteBuffers(1, &g_shapeMeshes[i].vbo);
    }
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

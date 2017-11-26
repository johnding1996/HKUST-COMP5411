THREE.ThinFilmShader = {

    vertexShader: [
        "const float PI = 3.1415926535897932384626433832795;",

        "uniform float mWaveLength[3];",
        "uniform float mAverageThickness;",
        "uniform float mThicknessRange;",
        "uniform float mRefractiveIndex;",
        "uniform float mEffectiveRelactiveIndex;",
        "uniform float mInterferenceMagnifier;",

        "varying vec3 vReflect;",
        "varying vec3 vRefract;",
        "varying float reflectivity[3];",

        "void main() {",

        "vec4 mvPosition = modelViewMatrix * vec4( position, 1.0 );",
        "vec4 worldPosition = modelMatrix * vec4( position, 1.0 );",
        "vec3 worldNormal = normalize(mat3(modelMatrix[0].xyz, modelMatrix[1].xyz, modelMatrix[2].xyz) * normal);",
        "vec3 I = worldPosition.xyz - cameraPosition;",

        "vReflect = reflect(I, worldNormal);",
        "vRefract = refract(normalize(I), worldNormal, mEffectiveRelactiveIndex);",

        "float cosine = dot(normalize(I), normalize(worldNormal));",
        "float sine = sqrt(1.0 - cosine * cosine);",
        "float effectiveCosine = sqrt(1.0 - sine * sine / mRefractiveIndex / mRefractiveIndex);",
        "float reflectivityS = (cosine - mRefractiveIndex * effectiveCosine)/(cosine + mRefractiveIndex * effectiveCosine);",
        "reflectivityS = reflectivityS * reflectivityS;",
        "float reflectivityP = (effectiveCosine - mRefractiveIndex * cosine)/(effectiveCosine + mRefractiveIndex * cosine);",
        "reflectivityP = reflectivityP * reflectivityP;",

        "float direction = fract(sin(dot(normalize(worldNormal).xy, vec2(1.0, 1.0)) * PI));",
        "float thickness = mAverageThickness * (1.0 - normalize(worldNormal).z * direction  * mThicknessRange);",
        "float cosineDelta[3];",
        "cosineDelta[0] = cos(4.0 * PI * mRefractiveIndex * thickness / mWaveLength[0] * cosine);",
        "cosineDelta[1] = cos(4.0 * PI * mRefractiveIndex * thickness / mWaveLength[1] * cosine);",
        "cosineDelta[2] = cos(4.0 * PI * mRefractiveIndex * thickness / mWaveLength[2] * cosine);",

        "reflectivity[0] = 2.0 * reflectivityS * reflectivityS * (1.0 - cosineDelta[0]) / " +
        "(1.0 + reflectivityS * reflectivityS * reflectivityS * reflectivityS - 2.0 * reflectivityS * reflectivityS * cosineDelta[0]);",
        "reflectivity[0] += 2.0 * reflectivityP * reflectivityP * (1.0 - cosineDelta[0]) / " +
        "(1.0 + reflectivityP * reflectivityP * reflectivityP * reflectivityP - 2.0 * reflectivityP * reflectivityP * cosineDelta[0]);",
        "reflectivity[1] = 2.0 * reflectivityS * reflectivityS * (1.0 - cosineDelta[1]) / " +
        "(1.0 + reflectivityS * reflectivityS * reflectivityS * reflectivityS - 2.0 * reflectivityS * reflectivityS * cosineDelta[1]);",
        "reflectivity[1] += 2.0 * reflectivityP * reflectivityP * (1.0 - cosineDelta[1]) / " +
        "(1.0 + reflectivityP * reflectivityP * reflectivityP * reflectivityP - 2.0 * reflectivityP * reflectivityP * cosineDelta[1]);",
        "reflectivity[2] = 2.0 * reflectivityS * reflectivityS * (1.0 - cosineDelta[2]) / " +
        "(1.0 + reflectivityS * reflectivityS * reflectivityS * reflectivityS - 2.0 * reflectivityS * reflectivityS * cosineDelta[2]);",
        "reflectivity[2] += 2.0 * reflectivityP * reflectivityP * (1.0 - cosineDelta[2]) / " +
        "(1.0 + reflectivityP * reflectivityP * reflectivityP * reflectivityP - 2.0 * reflectivityP * reflectivityP * cosineDelta[2]);",
        "reflectivity[0] *= mInterferenceMagnifier;",
        "reflectivity[1] *= mInterferenceMagnifier;",
        "reflectivity[2] *= mInterferenceMagnifier;",

        "gl_Position = projectionMatrix * mvPosition;",

        "}"

    ].join( "\n" ),

    fragmentShader: [

        "uniform samplerCube tCube;",

        "varying vec3 vReflect;",
        "varying vec3 vRefract;",
        "varying float reflectivity[3];",

        "void main() {",

        "vec4 reflectedColor = vec4(1.0);",
        "reflectedColor.r = textureCube(tCube, vec3(-vReflect.x, vReflect.yz)).r;",
        "reflectedColor.g = textureCube(tCube, vec3(-vReflect.x, vReflect.yz)).g;",
        "reflectedColor.b = textureCube(tCube, vec3(-vReflect.x, vReflect.yz)).b;",
        "vec4 refractedColor = vec4(1.0);",
        "refractedColor.r = textureCube(tCube, vec3(-vRefract.x, vRefract.yz)).r;",
        "refractedColor.g = textureCube(tCube, vec3(-vRefract.x, vRefract.yz)).g;",
        "refractedColor.b = textureCube(tCube, vec3(-vRefract.x, vRefract.yz)).b;",
        "vec4 color = vec4(1.0);",
        "color.r = mix(refractedColor.r, reflectedColor.r, clamp(reflectivity[0], .0, 1.0));",
        "color.g = mix(refractedColor.g, reflectedColor.g, clamp(reflectivity[1], .0, 1.0));",
        "color.b = mix(refractedColor.b, reflectedColor.b, clamp(reflectivity[2], .0, 1.0));",

        "gl_FragColor = color;",

        "}"

    ].join( "\n" )

};
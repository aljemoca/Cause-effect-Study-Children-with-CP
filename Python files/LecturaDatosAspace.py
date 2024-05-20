# -*- coding: utf-8 -*-
"""
Created on Wed Aug  2 19:11:25 2023

@author: alber
"""

#import numpy as np
import openpyxl





def SeleccionarCampo(Entrada,campo):
    try:
        pos_campo=Entrada.index(campo)
        Entrada=Entrada[pos_campo:]
        #print(Entrada)
        try:
            pos_coma = Entrada.index(',')
            if campo=='datos':
                pos_2coma = Entrada[pos_coma+1:].index(',')  #Busco LA SEGUNDA COMA
                pos_coma+=pos_2coma+1
        except Exception:
            pos_coma=len(Entrada)
            
        #print(Entrada)
        Entrada=Entrada[:pos_coma]
        pos_dp = Entrada.index(':')
        Entrada=Entrada[pos_dp+1:]
        #print(Entrada)
        return Entrada
    except Exception as e:
        print(f"Error {e} en SeleccionarCampo "+campo)
        return None

def ExtraerUsuario(Entrada):   
    cadena=SeleccionarCampo(Entrada,'nombre')
    cad = SeleccionarCampo(Entrada,'apellidos')
    return cadena + cad


def ExtraerNumeroEntrada(Entrada):
    #str_n = Entrada[:Entrada.index(',')]
    #cadena=str_n[str_n.index(':')+1:]
    cadena = SeleccionarCampo(Entrada,'pid')
    pid=int(cadena)
    return pid


def ExtraerDatos(Entrada):
    cadena = SeleccionarCampo(Entrada,'datos')
    evento=''
    tiempo=0
    posc = cadena.index(',')
    evento = cadena[:posc]

    #print(cadena)
    if len(cadena)>10:
        error=1
        tiempo = -1;
    else:
        if len(evento) != 2:
            error=1
        else:
            error=0
 #       posc = cadena.index(',')
  #      evento = cadena[:posc]
        #print(evento)
        #print(cadena[posc+1:])
        tiempo = int(cadena[posc+1:])
    return evento,tiempo,error


def ProcesarEntrada(Entrada):
    data=[]
    Entrada = Entrada.replace('"','')
    print(Entrada)
    pid=ExtraerNumeroEntrada(Entrada)
    usuario = ExtraerUsuario(Entrada)
    fecha = SeleccionarCampo(Entrada,'fecha')
    hora = SeleccionarCampo(Entrada,'hora')
    evento,tiempo,error = ExtraerDatos(Entrada)
    tipo = SeleccionarCampo(Entrada,'Exp')
    
    #if not error:
       # print(f'Pid={pid}, Usuario='+usuario+',Hora y día='+fecha+'/'+hora+',Evento='+evento+',tiempo='+str(tiempo)+',tipo='+tipo)    
    data=[pid, usuario, fecha, hora, evento, tiempo, tipo,error]
    print(data)
    return error,data


def ObtenerBaseDatos(contenido):
    #print(contenido)
    longitud=len(contenido)
    Error=0
    Ent=0
    Base=[]
    while longitud>10:
        RegIni = contenido.index('{')
        RegFin = contenido.index('}')
        Entrada= contenido[(RegIni+1):RegFin]
        a,b=ProcesarEntrada(Entrada)
        Error+=a
        Base.append(b)
        contenido=contenido[(RegFin+1):]
        longitud = len(contenido)
        Ent+= 1;
    return Error, Ent, Base

file_path = "AspaceOrdenadorSilla.txt"

try:
    with open(file_path, 'r') as file:
        contenido = file.read()
        # Crear un nuevo libro (hoja de cálculo)
        workbook = openpyxl.Workbook()

        # Seleccionar la hoja activa (por defecto es la primera)
        sheet = workbook.active
        sheet.append(['Indice','Usuario','Fecha','Hora','Estado','Tiempo','Tipo','Error'])
        # Agregar datos a la hoja de cálculo
        EntradasErroneas,Correctas,data=ObtenerBaseDatos(contenido)
       
        for row in data:
            sheet.append(row)

        # Guardar el libro en un archivo
        workbook.save("AspaceData.xlsx")
        #print(contenido)
        print('Entradas con errores='+str(EntradasErroneas))
        print('Entradas correctas='+str(Correctas))
except FileNotFoundError:
    print(f"El archivo '{file_path}' no fue encontrado.")

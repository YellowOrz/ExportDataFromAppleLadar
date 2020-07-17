// https://www.jianshu.com/p/6b5ca375c055
import Foundation
import AVFoundation
import UIKit

extension CVPixelBuffer {
    // 主要用于保存彩色图
    // 将CVPixelBuffer保存为PNG图片
    // https://stackoverflow.com/questions/37344822/saving-image-and-then-loading-it-in-swift-ios
    func SaveAsPNG(PNGName:String) -> Bool{
        let ciImage=CIImage(cvPixelBuffer: self)
        let uiImage=UIImage(ciImage: ciImage)
        guard let data = uiImage.pngData() else { return false }
        guard let directory = try? FileManager.default.url(for: .documentDirectory, in: .userDomainMask, appropriateFor: nil, create: false) as NSURL else {
            return false
        }
        do {
            try data.write(to: directory.appendingPathComponent(PNGName)!)
            print("Save \(PNGName) success!")
            return true
        } catch {
            print(error.localizedDescription)
            return false
        }
    }
    
    // 主要用于保存置信图
    // 将数值的格式为UInt8的CVPixelBuffer，通过矩阵形式一次性存储到二进制文件中
    func UInt8_Binary_Matrix(fileName:String) {
        let width = CVPixelBufferGetWidth(self)
        let height = CVPixelBufferGetHeight(self)

        CVPixelBufferLockBaseAddress(self, CVPixelBufferLockFlags(rawValue: 0))
        var floatBuffer = unsafeBitCast(CVPixelBufferGetBaseAddress(self), to: UnsafeMutablePointer<UInt8>.self)

        var i:Int = 0
        // 创建二维矩阵，存储每个像素点的值
        var Matrix = [[UInt8]]()//https://www.jianshu.com/p/23d4bc7c4f48
        for y in 0 ..< height {
            var row = [UInt8]()
            for x in 0 ..< width {
                var value:UInt8 = floatBuffer[Int(y * width + x)]
                row.append(value)
                i=i+1
            }
            Matrix.append(row)
        }
        // 保存到二进制文件中：https://stackoverflow.com/questions/33813812/write-array-of-float-to-binary-file-and-read-it-in-swift
        if let docsPath: String = NSSearchPathForDirectoriesInDomains(FileManager.SearchPathDirectory.documentDirectory, FileManager.SearchPathDomainMask.userDomainMask, true).last {
            let arrayPath:String = (((docsPath as NSString).appendingPathComponent(fileName) as NSString) as String)
            (Matrix as NSArray).write(toFile: arrayPath, atomically: false)
            print("Save \(fileName) success!")
        }
      
    }
    
    // 主要用于保存深度图
    // 将数值的格式为Float32的CVPixelBuffer，通过矩阵形式一次性存储到二进制文件中
    func Float32_Binary_Matrix(fileName:String) {
        let width = CVPixelBufferGetWidth(self)
        let height = CVPixelBufferGetHeight(self)

        CVPixelBufferLockBaseAddress(self, CVPixelBufferLockFlags(rawValue: 0))
        var floatBuffer = unsafeBitCast(CVPixelBufferGetBaseAddress(self), to: UnsafeMutablePointer<Float32>.self)

        var i:Int = 0
        // 创建二维矩阵，存储每个像素点的值
        var Matrix = [[Float32]]()//https://www.jianshu.com/p/23d4bc7c4f48
        for y in 0 ..< height {
            var row = [Float32]()
            for x in 0 ..< width {
                var value:Float32 = floatBuffer[Int(y * width + x)]
                row.append(value*1000)//将深度图的单位从m变成mm
                i=i+1
            }
            Matrix.append(row)
        }
        // 保存到二进制文件中：https://stackoverflow.com/questions/33813812/write-array-of-float-to-binary-file-and-read-it-in-swift
        if let docsPath: String = NSSearchPathForDirectoriesInDomains(FileManager.SearchPathDirectory.documentDirectory, FileManager.SearchPathDomainMask.userDomainMask, true).last {
            let arrayPath:String = (((docsPath as NSString).appendingPathComponent(fileName) as NSString) as String)
            (Matrix as NSArray).write(toFile: arrayPath, atomically: false)
            print("Save \(fileName) success!")
        }
      
    }
    
    // 不好用👎
    // 将CVPixelBuffer中的每一个值以此存入二进制文件（可能是二进制文件吧，毕竟资料都是网上查的）
    func SaveAsBinaryDirectly(BinName:String) -> Bool {
        let width = CVPixelBufferGetWidth(self)
        let height = CVPixelBufferGetHeight(self)

        CVPixelBufferLockBaseAddress(self, CVPixelBufferLockFlags(rawValue: 0))
        var floatBuffer = unsafeBitCast(CVPixelBufferGetBaseAddress(self), to: UnsafeMutablePointer<Float>.self)
        
        //方式一：转成string（不行）
//        // https://blog.csdn.net/Sico2Sico/article/details/79213122
//        let fileManager = FileManager.default
//        let file = NSSearchPathForDirectoriesInDomains(FileManager.SearchPathDirectory.documentDirectory, FileManager.SearchPathDomainMask.userDomainMask, true).first
//        let path = file! + BinName
//        fileManager.createFile(atPath: path, contents:nil, attributes:nil)
//        let handle = FileHandle(forWritingAtPath:path)
//        for y in 0 ..< height {
//            for x in 0 ..< width {
//                var pixel:Float = floatBuffer[y * width + x]
//                handle?.write(String(pixel*1000).data(using: String.Encoding.utf8)!)
//            }
//        }
//        print("Save \(BinName) success!")
//        return true
        
        //方式二：（不能正常保存文件，显示“Unable to write in new file.”）只打开关闭一次文件
//        // get path of directory
//        guard let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).last else {
//            return false
//        }
//        // create file url
//        let fileurl =  directory.appendingPathComponent("\(BinName).txt")
//        // if file exists then write data
//        if FileManager.default.fileExists(atPath: fileurl.path) {
//            if let fileHandle = FileHandle(forWritingAtPath: fileurl.path) {
//                // seekToEndOfFile, writes data at the last of file(appends not override)
//                for y in 0 ..< height {
//                    for x in 0 ..< width {
//                        var pixel:Float = floatBuffer[y * width + x]
//                        let data=Data(buffer: UnsafeBufferPointer(start: &pixel, count: 1))
//                        fileHandle.seekToEndOfFile()
//                        fileHandle.write(data)
//                    }
//                }
//                fileHandle.closeFile()
//                print("Save \(BinName) success!")
//                return true
//            }else {
//                print("Can't open file to write.")
//                return false
//            }
//        }else {
//            print("Unable to write in new file.")
//            return false
//        }
        
        //方式三：（肯定可用，但是一张要8秒）每获取一次数据，打开写入关闭文件一次
        for y in 0 ..< height {
            for x in 0 ..< width {
                var pixel:Float = floatBuffer[y * width + x]
                // https://stackoverflow.com/questions/36812583/how-to-convert-a-float-value-to-byte-array-in-swift
                let data=Data(buffer: UnsafeBufferPointer(start: &pixel, count: 1))
                writeToFile(data: data, fileName: BinName)
            }
        }
        print("Save \(BinName) success!")
        return true
    }
    
    // 不好用👎
    // 配合函数SaveAsBinaryDirectly()使用
    // https://stackoverflow.com/questions/57266115/how-to-write-bytes-data-in-file-swift
    func writeToFile(data: Data, fileName: String){
        // get path of directory
        guard let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).last else {
            return
        }
        // create file url
        let fileurl =  directory.appendingPathComponent("\(fileName).txt")
        // if file exists then write data
        if FileManager.default.fileExists(atPath: fileurl.path) {
            if let fileHandle = FileHandle(forWritingAtPath: fileurl.path) {
                // seekToEndOfFile, writes data at the last of file(appends not override)
                fileHandle.seekToEndOfFile()
                fileHandle.write(data)
                fileHandle.closeFile()
            }
            else {print("Can't open file to write.")}
        }
        else {
            // if file does not exist write data for the first time
            do{try data.write(to: fileurl, options: .atomic)}
            catch {print("Unable to write in new file.")}
        }
    }
   
}
